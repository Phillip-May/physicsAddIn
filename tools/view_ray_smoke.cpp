// Does a click become the right ray?

#include "ViewRay.cpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (condition) return;
    std::printf("  FAIL: %s\n", what);
    ++g_failures;
}

double length(const CadVec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

CadVec3 minus(const CadVec3& a, const CadVec3& b) {
    return CadVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

double dotp(const CadVec3& a, const CadVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// How far a point sits off a ray: the length of the component perpendicular to it.
double distanceToRay(const viewray::Ray& ray, const CadVec3& point) {
    const CadVec3 relative = minus(point, ray.origin);
    const double along = dotp(relative, ray.direction);
    const CadVec3 perpendicular(relative.x - ray.direction.x * along,
                                relative.y - ray.direction.y * along,
                                relative.z - ray.direction.z * along);
    return length(perpendicular);
}

// A camera looking from `eye` at `at`, built the way a view pose is: columns are the camera's own axes.
CadTransform lookAt(const CadVec3& eye, const CadVec3& at) {
    CadVec3 forward(at.x - eye.x, at.y - eye.y, at.z - eye.z);
    const double span = length(forward);
    forward = CadVec3(forward.x / span, forward.y / span, forward.z / span);
    CadVec3 worldUp(0.0, 0.0, 1.0);
    if (std::fabs(dotp(forward, worldUp)) > 0.99) worldUp = CadVec3(0.0, 1.0, 0.0);
    CadVec3 right(forward.y * worldUp.z - forward.z * worldUp.y,
                  forward.z * worldUp.x - forward.x * worldUp.z,
                  forward.x * worldUp.y - forward.y * worldUp.x);
    const double rightSpan = length(right);
    right = CadVec3(right.x / rightSpan, right.y / rightSpan, right.z / rightSpan);
    const CadVec3 up(right.y * forward.z - right.z * forward.y,
                     right.z * forward.x - right.x * forward.z,
                     right.x * forward.y - right.y * forward.x);
    // Camera looks down -Z, so its Z axis is backwards.
    CadTransform pose;
    pose.values[0] = right.x;  pose.values[1] = up.x;  pose.values[2] = -forward.x;  pose.values[3] = eye.x;
    pose.values[4] = right.y;  pose.values[5] = up.y;  pose.values[6] = -forward.y;  pose.values[7] = eye.y;
    pose.values[8] = right.z;  pose.values[9] = up.z;  pose.values[10] = -forward.z; pose.values[11] = eye.z;
    return pose;
}

viewray::Camera cameraAt(const CadVec3& eye, const CadVec3& at, int width, int height, double angle,
                         bool vertical) {
    viewray::Camera camera;
    camera.pose = lookAt(eye, at);
    camera.poseIsWorldFromCamera = true;
    camera.looksDownNegativeZ = true;
    camera.widthPx = width;
    camera.heightPx = height;
    camera.viewAngleDeg = angle;
    camera.angleIsVertical = vertical;
    return camera;
}

} // namespace

int main() {
    std::printf("=== view_ray_smoke ===\n");

    // RoboDK's own numbers: 30 deg, and the viewport the probe measured on this machine.
    const viewray::Camera camera =
        cameraAt(CadVec3(3000.0, -4000.0, 2500.0), CadVec3(0.0, 0.0, 400.0), 1918, 898, 30.0, true);

    std::printf("--- focal length\n");
    const double focal = viewray::focalLengthPx(camera);
    // 30 deg vertically over 898 px: (898/2) / tan(15 deg).
    const double expected = (898.0 / 2.0) / std::tan(15.0 * 3.14159265358979323846 / 180.0);
    std::printf("  focal = %.4f px (expected %.4f)\n", focal, expected);
    check(std::fabs(focal - expected) < 1.0e-6, "focal length matches the angle it came from");

    std::printf("--- the centre pixel looks straight ahead\n");
    const viewray::Ray centre = viewray::rayThroughPixel(camera, 1918.0 / 2.0, 898.0 / 2.0);
    check(distanceToRay(centre, CadVec3(0.0, 0.0, 400.0)) < 1.0e-6,
          "the centre pixel's ray passes through what the camera looks at");

    std::printf("--- project, then unproject: the ray must come back through the point\n");
    const CadVec3 points[] = {
        CadVec3(0.0, 0.0, 400.0),      CadVec3(500.0, 200.0, 900.0),
        CadVec3(-800.0, -300.0, 150.0), CadVec3(1200.0, 1100.0, 1600.0),
        CadVec3(-200.0, 900.0, 0.0),   CadVec3(300.0, -700.0, 2200.0),
    };
    double worst = 0.0;
    for (const CadVec3& point : points) {
        double pixelX = 0.0;
        double pixelY = 0.0;
        if (!viewray::projectPoint(camera, point, &pixelX, &pixelY)) {
            check(false, "a point in front of the camera projected");
            continue;
        }
        const viewray::Ray ray = viewray::rayThroughPixel(camera, pixelX, pixelY);
        const double off = distanceToRay(ray, point);
        worst = off > worst ? off : worst;
        std::printf("  (%7.1f, %7.1f, %7.1f) -> pixel (%7.2f, %7.2f) -> %.6f mm off its ray\n",
                    point.x, point.y, point.z, pixelX, pixelY, off);
    }
    check(worst < 1.0e-6, "every point lies on the ray through its own pixel");

    std::printf("--- and for cameras that are awkward on purpose\n");
    const viewray::Camera others[] = {
        cameraAt(CadVec3(0.0, 0.0, 9000.0), CadVec3(0.0, 0.0, 0.0), 640, 480, 60.0, true),
        cameraAt(CadVec3(-5000.0, 0.0, 100.0), CadVec3(0.0, 0.0, 100.0), 1000, 1000, 15.0, true),
        cameraAt(CadVec3(2000.0, 2000.0, 2000.0), CadVec3(-500.0, 300.0, 0.0), 1920, 1080, 30.0, false),
    };
    for (const viewray::Camera& other : others) {
        double off = 0.0;
        for (const CadVec3& point : points) {
            double pixelX = 0.0;
            double pixelY = 0.0;
            if (!viewray::projectPoint(other, point, &pixelX, &pixelY)) continue;
            const viewray::Ray ray = viewray::rayThroughPixel(other, pixelX, pixelY);
            const double here = distanceToRay(ray, point);
            off = here > off ? here : off;
        }
        std::printf("  %dx%d at %.0f deg %s: worst %.9f mm\n", other.widthPx, other.heightPx,
                    other.viewAngleDeg, other.angleIsVertical ? "vertical" : "horizontal", off);
        check(off < 1.0e-6, "round trip holds for this camera too");
    }

    std::printf("--- a horizontal reading of the angle is a different camera, and must be\n");
    const viewray::Camera vertical = cameraAt(CadVec3(3000.0, 0.0, 0.0), CadVec3(0.0, 0.0, 0.0),
                                              1918, 898, 30.0, true);
    viewray::Camera horizontal = vertical;
    horizontal.angleIsVertical = false;
    double vx = 0.0;
    double hx = 0.0;
    viewray::projectPoint(vertical, CadVec3(0.0, 300.0, 0.0), &vx, nullptr);
    viewray::projectPoint(horizontal, CadVec3(0.0, 300.0, 0.0), &hx, nullptr);
    std::printf("  same point: vertical reading %.2f px, horizontal reading %.2f px\n", vx, hx);
    check(std::fabs(vx - hx) > 100.0,
          "the two readings disagree by far more than a click's precision, so one screen test tells them "
          "apart");

    std::printf("--- ray against triangles\n");
    // One triangle spanning the origin, a metre in front of a camera looking down -X.
    const std::vector<float> triangle = {0.0f, -500.0f, -500.0f, 0.0f, 500.0f, -500.0f,
                                         0.0f, 0.0f,    500.0f};
    const viewray::Camera atTriangle =
        cameraAt(CadVec3(2000.0, 0.0, 0.0), CadVec3(0.0, 0.0, 0.0), 800, 600, 30.0, true);
    const viewray::Ray hit = viewray::rayThroughPixel(atTriangle, 400.0, 300.0);
    double distance = 0.0;
    check(viewray::rayHitsTriangles(hit, triangle, 1, &distance), "the centre ray hits the triangle");
    std::printf("  hit at %.3f mm (expected 2000)\n", distance);
    check(std::fabs(distance - 2000.0) < 1.0e-6, "at the distance the geometry says");

    const viewray::Ray miss = viewray::rayThroughPixel(atTriangle, 10.0, 10.0);
    check(!viewray::rayHitsTriangles(miss, triangle, 1, nullptr), "a corner ray misses it");

    std::printf("--- a silhouette conforms to the shape, and follows the eye\n");
    // A unit cube as a triangle soup, every vertex repeated the way generated geometry repeats them.
    std::vector<float> cube;
    {
        const double c[8][3] = {{-500, -500, -500}, {500, -500, -500}, {500, 500, -500}, {-500, 500, -500},
                                {-500, -500, 500},  {500, -500, 500},  {500, 500, 500},  {-500, 500, 500}};
        const int faces[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                                  {2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {0, 4, 7}, {0, 7, 3}};
        for (const auto& face : faces) {
            for (const int corner : face) {
                for (int axis = 0; axis < 3; ++axis) cube.push_back(static_cast<float>(c[corner][axis]));
            }
        }
    }
    const viewray::MeshEdges edges = viewray::buildMeshEdges(cube, 12);
    std::printf("  a cube of 12 triangles welds to %d edges (expected 18)\n", edges.edges);
    check(edges.edges == 18, "a cube has eighteen edges once coincident vertices are welded");

    std::vector<float> lines;
    // Face-on: the silhouette is the four edges of the face's outline.
    viewray::silhouetteLines(edges, CadVec3(9000.0, 0.0, 0.0), &lines);
    std::printf("  face-on: %d segments (expected 4)\n", static_cast<int>(lines.size() / 6));
    check(lines.size() / 6 == 4, "seen face-on, a cube outlines as a square");

    // Corner-on: six edges make the hexagonal outline.
    viewray::silhouetteLines(edges, CadVec3(9000.0, 9000.0, 9000.0), &lines);
    std::printf("  corner-on: %d segments (expected 6)\n", static_cast<int>(lines.size() / 6));
    check(lines.size() / 6 == 6, "seen corner-on, it outlines as a hexagon");

    bool onTheCube = true;
    for (size_t at = 0; at + 5 < lines.size(); at += 6) {
        for (int end = 0; end < 2; ++end) {
            for (int axis = 0; axis < 3; ++axis) {
                const float value = lines[at + end * 3 + axis];
                if (std::fabs(std::fabs(value) - 500.0f) > 1.0e-3f) onTheCube = false;
            }
        }
    }
    check(onTheCube, "every silhouette segment lies on the shape rather than around it");

    if (g_failures == 0) {
        std::printf("\nPASS: a pixel becomes a ray, and the ray hits what it points at.\n");
        return 0;
    }
    std::printf("\nFAIL: %d check(s) failed.\n", g_failures);
    return 1;
}
