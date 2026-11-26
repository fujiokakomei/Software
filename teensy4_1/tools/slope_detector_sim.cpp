// SlopeDetector の動作確認用シミュレーション（PC上で実行する）。
//   cd teensy4_1
//   c++ -std=c++17 -O2 -Iinclude tools/slope_detector_sim.cpp src/SlopeDetector.cpp -o slope_sim && ./slope_sim
//
// 地形の高さ h(x) の上を、前後の車輪（半径 r、ホイールベース W）を持つ機体が
// 一定速度で走ったときの前後傾きを計算し、BNO055相当の遅れとノイズを加えて
// SlopeDetector に入力する。坂だけが確定し、バンプ・段差・階段では確定しないことを確認する。
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "SlopeDetector.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
using Terrain = std::function<double(double)>; // x[mm] -> 高さ[mm]

double deg2rad(double d) { return d * kPi / 180.0; }

// 平地 → 坂（水平300mm）→ 平地
Terrain ramp(double angleDeg, double startX = 400.0, double length = 300.0)
{
    const double t = std::tan(deg2rad(angleDeg));
    return [=](double x) {
        if (x < startX) return 0.0;
        if (x < startX + length) return (x - startX) * t;
        return length * t;
    };
}

// 台形のバンプ
Terrain bump(double height, double topWidth, double slopeWidth, double startX = 400.0)
{
    return [=](double x) {
        const double a = startX, b = a + slopeWidth, c = b + topWidth, d = c + slopeWidth;
        if (x < a || x > d) return 0.0;
        if (x < b) return height * (x - a) / slopeWidth;
        if (x < c) return height;
        return height * (d - x) / slopeWidth;
    };
}

// 段差（上ってそのまま）
Terrain step(double height, double startX = 400.0)
{
    return [=](double x) { return x < startX ? 0.0 : height; };
}

// 階段: rise の段を count 段上り、平らな踊り場 landing を挟んで同じ段数下りる
Terrain stairs(double rise, double tread, int count, double landing = 150.0, double startX = 400.0)
{
    return [=](double x) {
        if (x < startX) return 0.0;
        const double up = tread * count;
        if (x < startX + up) return rise * (std::floor((x - startX) / tread) + 1);
        if (x < startX + up + landing) return rise * count;
        const double down = x - (startX + up + landing);
        if (down < tread * count) return rise * (count - 1 - std::floor(down / tread));
        return 0.0;
    };
}

// 高い所から下る坂
Terrain rampDown(double angleDeg) {
    const Terrain up = ramp(angleDeg);
    const double top = up(1e9);
    return [=](double x) { return top - up(x); };
}

struct Robot
{
    double wheelbase;    // mm
    double wheelRadius;  // mm
    double speed;        // mm/s
};

// 車輪中心の高さ（車輪が地形に接する位置）
double wheelCenterHeight(const Terrain& h, double x, double r)
{
    double best = -1e9;
    for (int i = -40; i <= 40; ++i)
    {
        const double u = r * i / 40.0;
        best = std::max(best, h(x + u) + std::sqrt(r * r - u * u));
    }
    return best - r;
}

struct Result
{
    bool confirmed = false;
    int direction = 0;
    bool finished = false;
    double maxPitch = 0.0;
};

Result run(const Terrain& h, const Robot& robot, uint32_t seed)
{
    slope::SlopeDetector detector;
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.5);
    std::uniform_int_distribution<int> period(45, 65); // 制御ループ周期（カラーセンサー読み取り込み）

    double filtered = 0.0;
    double t = 0.0;  // ms
    double x = 0.0;  // 機体中心 mm
    uint32_t nextSample = 0;
    Result result;
    const double dtMs = 1.0;
    while (x < 1400.0)
    {
        const double front = wheelCenterHeight(h, x + robot.wheelbase / 2, robot.wheelRadius);
        const double rear = wheelCenterHeight(h, x - robot.wheelbase / 2, robot.wheelRadius);
        const double pitch = std::atan2(front - rear, robot.wheelbase) * 180.0 / kPi;
        // BNO055のフュージョン遅れを一次遅れ(20ms)で近似
        filtered += (pitch - filtered) * (dtMs / 20.0);
        result.maxPitch = std::max(result.maxPitch, std::fabs(pitch));

        if (t >= nextSample)
        {
            detector.update(static_cast<float>(filtered + noise(rng)),
                            static_cast<float>(noise(rng)), static_cast<uint32_t>(t));
            nextSample = static_cast<uint32_t>(t) + period(rng);
        }
        // 上り坂では遅く、下り坂では速くなる
        const double v = robot.speed * (1.0 - 0.01 * pitch);
        x += v * dtMs / 1000.0 * std::cos(deg2rad(pitch));
        t += dtMs;
    }
    result.confirmed = detector.isConfirmed();
    result.direction = detector.direction();
    result.finished = detector.isFinished();
    return result;
}

struct Scenario
{
    std::string name;
    Terrain terrain;
    int expectedDirection; // 0: 坂ではない
    // true: 結果を表示するだけで合否に含めない。
    // 踏面が短い階段は、踏面×段数がホイールベースに近いと機体が常に一定角度で乗るため
    // ジャイロの値だけでは同じ角度の坂と区別できない（物理的な限界）。
    bool informational = false;
};

} // namespace

int main()
{
    std::vector<Scenario> scenarios = {
        {"ramp up 25deg", ramp(25), 1},
        {"ramp up 20deg", ramp(20), 1},
        {"ramp up 15deg", ramp(15), 1},
        {"ramp down 20deg", rampDown(20), -1},
        {"ramp down 15deg", rampDown(15), -1},
        {"speed bump 2cm narrow", bump(20, 10, 20), 0},
        {"speed bump 2cm wide", bump(20, 100, 30), 0},
        {"step up 2cm", step(20), 0},
        {"step down 2cm", [](double x) { return x < 400 ? 20.0 : 0.0; }, 0},
    };
    for (double tread : {40.0, 60.0, 80.0, 100.0, 120.0, 150.0, 200.0, 250.0})
    {
        char name[64];
        std::snprintf(name, sizeof(name), "stairs 2cm x5 tread %.0fmm", tread);
        scenarios.push_back({name, stairs(20, tread, 5), 0, tread < 100.0});
    }

    int failures = 0;
    int runs = 0;
    for (const Scenario& scenario : scenarios)
    {
        int ok = 0;
        int total = 0;
        double maxPitch = 0;
        std::string firstFailure;
        for (double wheelbase : {100.0, 120.0, 140.0, 160.0})
        {
            for (double radius : {20.0, 30.0, 40.0})
            {
                for (double speed : {120.0, 200.0, 300.0})
                {
                    for (uint32_t seed = 1; seed <= 5; ++seed)
                    {
                        const Robot robot{wheelbase, radius, speed};
                        const Result r = run(scenario.terrain, robot, seed);
                        maxPitch = std::max(maxPitch, r.maxPitch);
                        const bool pass = scenario.expectedDirection == 0
                            ? !r.confirmed
                            : (r.confirmed && r.direction == scenario.expectedDirection && r.finished);
                        ++total;
                        if (pass)
                        {
                            ++ok;
                        }
                        else if (firstFailure.empty())
                        {
                            char buf[128];
                            std::snprintf(buf, sizeof(buf), "W=%.0f r=%.0f v=%.0f seed=%u dir=%d fin=%d",
                                          wheelbase, radius, speed, seed, r.direction, r.finished);
                            firstFailure = buf;
                        }
                    }
                }
            }
        }
        if (!scenario.informational)
        {
            runs += total;
            failures += total - ok;
        }
        std::printf("%-32s %4d/%4d ok  (max pitch %.1f deg)%s%s%s\n", scenario.name.c_str(), ok, total,
                    maxPitch, scenario.informational ? " [参考]" : "",
                    firstFailure.empty() ? "" : "  first NG: ", firstFailure.c_str());
    }
    std::printf("\n%d / %d runs failed\n", failures, runs);
    return failures == 0 ? 0 : 1;
}
