// Proves RobotSimulator/SceneMath.h reproduces the Qt maths the renderer used.
#include <SceneMath.h>

#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int g_failures = 0;
float g_worstMatrix = 0.0f;
float g_worstVector = 0.0f;

constexpr float kTolerance = 1.0e-5f;

void compareMatrix(const char* label, const Mat4& mine, const QMatrix4x4& qt) {
    const float* a = mine.constData();
    const float* b = qt.constData();
    float worst = 0.0f;
    for (int i = 0; i < 16; ++i) {
        worst = std::max(worst, std::fabs(a[i] - b[i]) / std::max(1.0f, std::fabs(b[i])));
    }
    g_worstMatrix = std::max(g_worstMatrix, worst);
    if (worst > kTolerance) {
        std::printf("FAIL %s: worst relative element delta %g\n", label, worst);
        std::printf("   ours:");
        for (int i = 0; i < 16; ++i) std::printf(" %.6f", a[i]);
        std::printf("\n   qt  :");
        for (int i = 0; i < 16; ++i) std::printf(" %.6f", b[i]);
        std::printf("\n");
        ++g_failures;
    }
}

void compareVector(const char* label, const Vec3f& mine, const QVector3D& qt) {
    const float worst = std::max({std::fabs(mine.x() - qt.x()),
                                  std::fabs(mine.y() - qt.y()),
                                  std::fabs(mine.z() - qt.z())});
    g_worstVector = std::max(g_worstVector, worst);
    if (worst > kTolerance) {
        std::printf("FAIL %s: (%g,%g,%g) vs qt (%g,%g,%g)\n", label,
                    mine.x(), mine.y(), mine.z(), qt.x(), qt.y(), qt.z());
        ++g_failures;
    }
}

} // namespace

int main() {
    // --- the camera the viewer builds, over a sweep of realistic states ---
    for (float distance : {50.0f, 500.0f, 1000.0f, 12345.0f}) {
        for (float pitch : {-89.0f, -30.0f, 0.0f, 20.0f, 75.0f}) {
            for (float yaw : {-180.0f, -35.0f, 0.0f, 90.0f, 179.0f}) {
                for (float aspect : {0.5f, 1.0f, 1.7777f, 3.0f}) {
                    const Vec3f centre(123.5f, -45.25f, 678.125f);
                    const QVector3D qtCentre(123.5f, -45.25f, 678.125f);

                    Mat4 projection;
                    projection.setToIdentity();
                    projection.perspective(45.0f, aspect, 1.0f, 100000.0f);
                    QMatrix4x4 qtProjection;
                    qtProjection.setToIdentity();
                    qtProjection.perspective(45.0f, aspect, 1.0f, 100000.0f);
                    compareMatrix("projection", projection, qtProjection);

                    Mat4 view;
                    view.setToIdentity();
                    view.translate(0.0f, 0.0f, -distance);
                    view.rotate(pitch, 1.0f, 0.0f, 0.0f);
                    view.rotate(yaw, 0.0f, 1.0f, 0.0f);
                    view.translate(-centre.x(), -centre.y(), -centre.z());

                    QMatrix4x4 qtView;
                    qtView.setToIdentity();
                    qtView.translate(0.0f, 0.0f, -distance);
                    qtView.rotate(pitch, 1.0f, 0.0f, 0.0f);
                    qtView.rotate(yaw, 0.0f, 1.0f, 0.0f);
                    qtView.translate(-qtCentre.x(), -qtCentre.y(), -qtCentre.z());
                    compareMatrix("view", view, qtView);

                    Mat4 model;
                    QMatrix4x4 qtModel;
                    const float rigid[12] = {0.36f, -0.48f, 0.8f, 100.0f,
                                             0.8f,  0.6f,   0.0f, -25.0f,
                                             -0.48f, 0.64f, 0.6f, 3.5f};
                    model.setToIdentity();
                    qtModel.setToIdentity();
                    for (int row = 0; row < 3; ++row) {
                        for (int column = 0; column < 4; ++column) {
                            model.set(row, column, rigid[row * 4 + column]);
                            qtModel(row, column) = rigid[row * 4 + column];
                        }
                    }
                    compareMatrix("model", model, qtModel);

                    const Mat4 mvp = projection * view * model;
                    const QMatrix4x4 qtMvp = qtProjection * qtView * qtModel;
                    compareMatrix("mvp", mvp, qtMvp);

                    // projectPoint: clip space, the perspective divide, and the screen mapping.
                    for (const Vec3f& point : {Vec3f(0.0f, 0.0f, 0.0f), Vec3f(250.0f, -80.0f, 400.0f),
                                               Vec3f(-1000.0f, 900.0f, -50.0f)}) {
                        const Vec4f clip = (projection * view) * Vec4f(point, 1.0f);
                        const QVector4D qtClip =
                            qtProjection * qtView * QVector4D(point.x(), point.y(), point.z(), 1.0f);
                        const float deltas[4] = {std::fabs(clip.x() - qtClip.x()),
                                                 std::fabs(clip.y() - qtClip.y()),
                                                 std::fabs(clip.z() - qtClip.z()),
                                                 std::fabs(clip.w() - qtClip.w())};
                        const float magnitudes[4] = {std::fabs(qtClip.x()), std::fabs(qtClip.y()),
                                                     std::fabs(qtClip.z()), std::fabs(qtClip.w())};
                        for (int i = 0; i < 4; ++i) {
                            const float relative = deltas[i] / std::max(1.0f, magnitudes[i]);
                            if (relative > kTolerance) {
                                std::printf("FAIL clip component %d: delta %g (relative %g)\n",
                                            i, deltas[i], relative);
                                ++g_failures;
                            }
                        }
                    }
                }
            }
        }
    }

    // --- vector operations ---
    const Vec3f samples[] = {Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, -3.5f, 2.25f),
                             Vec3f(-120.5f, 45.0f, 8.125f), Vec3f(0.001f, 0.002f, 0.003f),
                             Vec3f(0.0f, 0.0f, 0.0f)};
    for (const Vec3f& a : samples) {
        const QVector3D qtA(a.x(), a.y(), a.z());
        compareVector("normalized", a.normalized(), qtA.normalized());
        if (std::fabs(a.length() - qtA.length()) > kTolerance) {
            std::printf("FAIL length: %g vs %g\n", a.length(), qtA.length());
            ++g_failures;
        }
        for (const Vec3f& b : samples) {
            const QVector3D qtB(b.x(), b.y(), b.z());
            compareVector("cross", Vec3f::crossProduct(a, b), QVector3D::crossProduct(qtA, qtB));
            const float dot = Vec3f::dotProduct(a, b);
            const float qtDot = QVector3D::dotProduct(qtA, qtB);
            if (std::fabs(dot - qtDot) > kTolerance * std::max(1.0f, std::fabs(qtDot))) {
                std::printf("FAIL dot: %g vs %g\n", dot, qtDot);
                ++g_failures;
            }
            compareVector("add", a + b, qtA + qtB);
            compareVector("sub", a - b, qtA - qtB);
        }
        compareVector("negate", -a, -qtA);
        compareVector("scale", a * 2.5f, qtA * 2.5f);
    }

    std::printf("\nworst matrix element delta: %g\nworst vector delta: %g\n",
                g_worstMatrix, g_worstVector);
    std::printf("%s\n", g_failures == 0 ? "PASS: SceneMath matches Qt" : "FAIL");
    return g_failures == 0 ? 0 : 1;
}
