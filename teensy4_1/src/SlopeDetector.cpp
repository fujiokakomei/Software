#include "SlopeDetector.hpp"

namespace slope
{

namespace
{

float absf(float value)
{
    return value < 0.0f ? -value : value;
}

} // namespace

void SlopeDetector::reset()
{
    _state = State::Flat;
    _direction = 0;
    _candidateSamples = 0;
    _hasLastSample = false;
    _flatTiming = false;
    _trackingTilt = false;
    _tiltDecreasing = false;
}

void SlopeDetector::reject(uint32_t nowMs)
{
    (void)nowMs;
    _state = State::Rejected;
    _direction = 0;
    _candidateSamples = 0;
    _flatTiming = false;
}

bool SlopeDetector::detectOscillation(float absPitch)
{
    if (!_trackingTilt)
    {
        _trackingTilt = true;
        _tiltDecreasing = false;
        _tiltPeak = absPitch;
        return false;
    }

    if (!_tiltDecreasing)
    {
        if (absPitch > _tiltPeak)
        {
            _tiltPeak = absPitch;
        }
        else if (absPitch < _tiltPeak - _config.maxPitchSpreadDeg)
        {
            _tiltDecreasing = true;
            _tiltValley = absPitch;
        }
        return false;
    }

    if (absPitch < _tiltValley)
    {
        _tiltValley = absPitch;
        return false;
    }
    // 一度大きく減った傾きが、平地に戻らないまま再び大きく増えた
    return absPitch > _tiltValley + _config.maxPitchSpreadDeg;
}

void SlopeDetector::startCandidate(float pitch, int sign, uint32_t nowMs)
{
    _state = State::Candidate;
    _direction = sign;
    _candidateSamples = 0;
    pushCandidateSample(pitch, nowMs);
}

void SlopeDetector::pushCandidateSample(float pitch, uint32_t nowMs)
{
    _historyPitch[_historyHead] = pitch;
    _historyMs[_historyHead] = nowMs;
    _historyHead = (_historyHead + 1) % kHistorySize;
    if (_candidateSamples < kHistorySize)
    {
        ++_candidateSamples;
    }
}

void SlopeDetector::update(float noseUpPitch, float sideRoll, uint32_t nowMs)
{
    const bool largeGap = _hasLastSample && nowMs - _lastSampleMs > _config.maxSampleGapMs;
    _hasLastSample = true;
    _lastSampleMs = nowMs;

    if (_state == State::Finished)
    {
        return;
    }

    const float absPitch = absf(noseUpPitch);
    const bool flat = absPitch < _config.exitPitchDeg;

    if (_state == State::Rejected)
    {
        // 平地に戻ったことを確認してから再び判定する
        if (flat)
        {
            if (!_flatTiming)
            {
                _flatTiming = true;
                _flatStartMs = nowMs;
            }
            if (nowMs - _flatStartMs >= _config.flatConfirmMs)
            {
                _state = State::Flat;
                _flatTiming = false;
                _trackingTilt = false;
            }
        }
        else
        {
            _flatTiming = false;
        }
        return;
    }

    if (flat)
    {
        _trackingTilt = false;
    }
    else if (detectOscillation(absPitch) &&
             (_state == State::Candidate || _state == State::OnSlope))
    {
        reject(nowMs);
        return;
    }

    if (_state == State::OnSlope)
    {
        if (flat)
        {
            if (!_flatTiming)
            {
                _flatTiming = true;
                _flatStartMs = nowMs;
            }
            if (nowMs - _flatStartMs >= _config.flatConfirmMs)
            {
                _state = State::Finished;
            }
        }
        else
        {
            _flatTiming = false;
        }
        return;
    }

    // Flat / Candidate
    const bool tilted = absPitch >= _config.enterPitchDeg &&
                        absf(sideRoll) <= _config.maxRollDeg;
    if (!tilted)
    {
        _state = State::Flat;
        _direction = 0;
        _candidateSamples = 0;
        return;
    }

    const int sign = noseUpPitch > 0.0f ? 1 : -1;
    if (_state != State::Candidate || sign != _direction || largeGap)
    {
        startCandidate(noseUpPitch, sign, nowMs);
        return;
    }

    pushCandidateSample(noseUpPitch, nowMs);

    // 最新のサンプルから遡り、ばらつきが maxPitchSpreadDeg 以内の区間を求める
    float minPitch = noseUpPitch;
    float maxPitch = noseUpPitch;
    uint32_t stableSinceMs = nowMs;
    int stableSamples = 0;
    for (int i = 0; i < _candidateSamples; ++i)
    {
        const int index = (_historyHead - 1 - i + kHistorySize) % kHistorySize;
        const float pitch = _historyPitch[index];
        const float nextMin = pitch < minPitch ? pitch : minPitch;
        const float nextMax = pitch > maxPitch ? pitch : maxPitch;
        if (nextMax - nextMin > _config.maxPitchSpreadDeg)
        {
            break;
        }
        minPitch = nextMin;
        maxPitch = nextMax;
        stableSinceMs = _historyMs[index];
        ++stableSamples;
    }

    if (nowMs - stableSinceMs >= _config.confirmMs && stableSamples >= _config.minSamples)
    {
        _state = State::OnSlope;
        _flatTiming = false;
    }
}

} // namespace slope
