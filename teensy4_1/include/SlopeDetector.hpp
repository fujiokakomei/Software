#pragma once

#include <stdint.h>

namespace slope
{

// ジャイロ(BNO055)の前後傾きだけで坂を判定する。
//
// 坂と、階段・バンプ（スピードバンプ、段差）は次の特徴で区別する。
//  1. 角度: 坂は機体全体が坂の角度まで傾く。2cm程度の段差・バンプでは
//     「段差の高さ / ホイールベース」程度しか傾かないので enterPitchDeg 未満になる。
//  2. 安定性: 坂の上では傾きがほぼ一定のまま続く。直近のサンプルを遡り、
//     ばらつき(最大-最小)が maxPitchSpreadDeg 以内に収まっている区間の長さを調べる。
//  3. 継続時間: その区間が confirmMs 以上、minSamples 個以上あれば坂の上にいると判断する。
//  4. 左右の傾き: 片輪だけ障害物やバンプに乗ったときは左右にも傾くので除外する。
//  5. 振動: 傾いたまま「傾きが減ってまた増える」のは階段の段を越えている証拠なので、
//     候補中・坂の走行中どちらでも見つけた時点で棄却し(Rejected)、平地に戻るまで再判定しない。
//     坂の入口では傾きは単調に増え、出口では単調に減るだけなので棄却されない。
//
// 坂の確定(OnSlope)後は、傾きが exitPitchDeg 未満の状態が flatConfirmMs 続いたら
// 坂を抜けた(Finished)とみなす。坂として地図に登録するのは Finished になってから。
class SlopeDetector
{
public:
    struct Config
    {
        float enterPitchDeg = 12.0f;     // これ以上傾いたら坂の候補
        float exitPitchDeg = 5.0f;       // これ未満なら平地
        float maxRollDeg = 8.0f;         // 候補中に許す左右の傾き
        float maxPitchSpreadDeg = 4.0f;  // 候補中に許す傾きのばらつき(最大-最小)
        uint32_t confirmMs = 400;        // 安定して傾いている必要がある時間
        uint32_t flatConfirmMs = 250;    // 坂を抜けたと判断するまでの平地の継続時間
        uint32_t maxSampleGapMs = 300;   // これ以上サンプル間隔が空いたら候補をやり直す
        int minSamples = 4;              // 確定に必要な最小サンプル数
    };

    enum class State
    {
        Flat,      // 平地（または坂でない傾き）
        Candidate, // 坂かもしれない傾きが続いている
        OnSlope,   // 安定した傾きを確認、坂の上を走行中（まだ階段の可能性は残る）
        Finished,  // 坂を抜けて平地に戻った（坂で確定）
        Rejected   // 階段などと判断。平地に戻るまで判定しない
    };

    SlopeDetector() = default;
    explicit SlopeDetector(const Config& config) : _config(config) {}

    void reset();
    // noseUpPitch: 機首上げが正の前後傾き[deg]、sideRoll: 左右の傾き[deg]
    void update(float noseUpPitch, float sideRoll, uint32_t nowMs);

    State state() const { return _state; }
    bool isCandidate() const { return _state == State::Candidate; }
    bool isOnSlope() const { return _state == State::OnSlope; }
    bool isConfirmed() const { return _state == State::OnSlope || _state == State::Finished; }
    bool isFinished() const { return _state == State::Finished; }
    bool isRejected() const { return _state == State::Rejected; }
    // +1: 上り坂、-1: 下り坂、0: 未確定
    int direction() const { return isConfirmed() ? _direction : 0; }
    const Config& config() const { return _config; }

private:
    void startCandidate(float pitch, int sign, uint32_t nowMs);
    void pushCandidateSample(float pitch, uint32_t nowMs);
    void reject(uint32_t nowMs);
    // 傾いている間の |傾き| の推移から「減ってから増えた」を検出する
    bool detectOscillation(float absPitch);

    Config _config;
    State _state = State::Flat;
    int _direction = 0;

    // 候補中のサンプル履歴（新しい方から遡って、ばらつきが小さい区間の長さを調べる）
    static constexpr int kHistorySize = 32;
    float _historyPitch[kHistorySize] = {};
    uint32_t _historyMs[kHistorySize] = {};
    int _historyHead = 0;  // 次に書き込む位置
    int _candidateSamples = 0; // 履歴に入っている数（最大 kHistorySize）

    bool _hasLastSample = false;
    uint32_t _lastSampleMs = 0;

    bool _flatTiming = false;
    uint32_t _flatStartMs = 0;

    bool _trackingTilt = false;
    bool _tiltDecreasing = false;
    float _tiltPeak = 0.0f;   // 増加中の最大値
    float _tiltValley = 0.0f; // 減少中の最小値
};

} // namespace slope
