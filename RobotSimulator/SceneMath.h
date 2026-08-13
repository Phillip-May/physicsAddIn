#pragma once

#include <cmath>

// Vector and matrix types for the scene renderer, replacing QVector3D, QVector4D,
// QMatrix4x4, QPoint and QPointF.

struct Vec3f {
    constexpr Vec3f() = default;
    constexpr Vec3f(float x, float y, float z) : m_x(x), m_y(y), m_z(z) {}

    constexpr float x() const { return m_x; }
    constexpr float y() const { return m_y; }
    constexpr float z() const { return m_z; }

    void setX(float value) { m_x = value; }
    void setY(float value) { m_y = value; }
    void setZ(float value) { m_z = value; }

    float length() const { return std::sqrt(m_x * m_x + m_y * m_y + m_z * m_z); }

    Vec3f normalized() const {
        const float len = length();
        if (len < 1.0e-30f) return Vec3f();
        return Vec3f(m_x / len, m_y / len, m_z / len);
    }

    static float dotProduct(const Vec3f& a, const Vec3f& b) {
        return a.m_x * b.m_x + a.m_y * b.m_y + a.m_z * b.m_z;
    }

    static Vec3f crossProduct(const Vec3f& a, const Vec3f& b) {
        return Vec3f(a.m_y * b.m_z - a.m_z * b.m_y,
                     a.m_z * b.m_x - a.m_x * b.m_z,
                     a.m_x * b.m_y - a.m_y * b.m_x);
    }

private:
    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_z = 0.0f;
};

inline Vec3f operator+(const Vec3f& a, const Vec3f& b) {
    return Vec3f(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
}
inline Vec3f operator-(const Vec3f& a, const Vec3f& b) {
    return Vec3f(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
}
inline Vec3f operator-(const Vec3f& v) { return Vec3f(-v.x(), -v.y(), -v.z()); }
inline Vec3f operator*(const Vec3f& v, float s) { return Vec3f(v.x() * s, v.y() * s, v.z() * s); }
inline Vec3f operator*(float s, const Vec3f& v) { return v * s; }
inline Vec3f operator/(const Vec3f& v, float s) { return Vec3f(v.x() / s, v.y() / s, v.z() / s); }
inline Vec3f& operator+=(Vec3f& a, const Vec3f& b) { a = a + b; return a; }
inline Vec3f& operator-=(Vec3f& a, const Vec3f& b) { a = a - b; return a; }

struct Vec4f {
    constexpr Vec4f() = default;
    constexpr Vec4f(float x, float y, float z, float w) : m_x(x), m_y(y), m_z(z), m_w(w) {}
    constexpr Vec4f(const Vec3f& v, float w) : m_x(v.x()), m_y(v.y()), m_z(v.z()), m_w(w) {}

    constexpr float x() const { return m_x; }
    constexpr float y() const { return m_y; }
    constexpr float z() const { return m_z; }
    constexpr float w() const { return m_w; }

private:
    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_z = 0.0f;
    float m_w = 0.0f;
};

// Integer and floating point screen coordinates, replacing QPoint and QPointF.
struct PointI {
    constexpr PointI() = default;
    constexpr PointI(int x, int y) : m_x(x), m_y(y) {}
    constexpr int x() const { return m_x; }
    constexpr int y() const { return m_y; }

private:
    int m_x = 0;
    int m_y = 0;
};

inline PointI operator-(const PointI& a, const PointI& b) {
    return PointI(a.x() - b.x(), a.y() - b.y());
}

struct PointF {
    constexpr PointF() = default;
    constexpr PointF(double x, double y) : m_x(x), m_y(y) {}
    constexpr double x() const { return m_x; }
    constexpr double y() const { return m_y; }

private:
    double m_x = 0.0;
    double m_y = 0.0;
};

inline PointF operator+(const PointF& a, const PointF& b) {
    return PointF(a.x() + b.x(), a.y() + b.y());
}
inline PointF operator-(const PointF& a, const PointF& b) {
    return PointF(a.x() - b.x(), a.y() - b.y());
}
inline PointF operator*(const PointF& p, double s) { return PointF(p.x() * s, p.y() * s); }
inline PointF operator*(double s, const PointF& p) { return p * s; }
inline PointF operator/(const PointF& p, double s) { return PointF(p.x() / s, p.y() / s); }
inline PointF& operator/=(PointF& p, double s) { p = p / s; return p; }
inline PointF& operator+=(PointF& a, const PointF& b) { a = a + b; return a; }

// 4x4 float matrix, column-major so constData() can go straight to glUniformMatrix4fv with
// transpose = GL_FALSE, exactly as QMatrix4x4 did.
class Mat4 {
public:
    Mat4() { setToIdentity(); }

    // Row-major argument order, matching QMatrix4x4's 16-float constructor: the first four
    // arguments are the top row. Storage is still column-major, so the values are transposed
    // on the way in.
    Mat4(float m11, float m12, float m13, float m14,
         float m21, float m22, float m23, float m24,
         float m31, float m32, float m33, float m34,
         float m41, float m42, float m43, float m44) {
        m_data[0]  = m11; m_data[4]  = m12; m_data[8]  = m13; m_data[12] = m14;
        m_data[1]  = m21; m_data[5]  = m22; m_data[9]  = m23; m_data[13] = m24;
        m_data[2]  = m31; m_data[6]  = m32; m_data[10] = m33; m_data[14] = m34;
        m_data[3]  = m41; m_data[7]  = m42; m_data[11] = m43; m_data[15] = m44;
    }

    void setToIdentity() {
        for (int i = 0; i < 16; ++i) m_data[i] = 0.0f;
        m_data[0] = m_data[5] = m_data[10] = m_data[15] = 1.0f;
    }

    const float* constData() const { return m_data; }

    // element(row, column)
    float at(int row, int column) const { return m_data[column * 4 + row]; }
    void set(int row, int column, float value) { m_data[column * 4 + row] = value; }

    Mat4 operator*(const Mat4& rhs) const {
        Mat4 result;
        for (int column = 0; column < 4; ++column) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) sum += at(row, k) * rhs.at(k, column);
                result.set(row, column, sum);
            }
        }
        return result;
    }

    Vec4f operator*(const Vec4f& v) const {
        return Vec4f(at(0, 0) * v.x() + at(0, 1) * v.y() + at(0, 2) * v.z() + at(0, 3) * v.w(),
                     at(1, 0) * v.x() + at(1, 1) * v.y() + at(1, 2) * v.z() + at(1, 3) * v.w(),
                     at(2, 0) * v.x() + at(2, 1) * v.y() + at(2, 2) * v.z() + at(2, 3) * v.w(),
                     at(3, 0) * v.x() + at(3, 1) * v.y() + at(3, 2) * v.z() + at(3, 3) * v.w());
    }

    void translate(float x, float y, float z) {
        Mat4 translation;
        translation.set(0, 3, x);
        translation.set(1, 3, y);
        translation.set(2, 3, z);
        *this = *this * translation;
    }

    // Angle in degrees about the given axis, as QMatrix4x4::rotate takes it.
    void rotate(float angleDegrees, float x, float y, float z) {
        const float lengthSquared = x * x + y * y + z * z;
        if (lengthSquared < 1.0e-30f) return;
        if (std::fabs(lengthSquared - 1.0f) > 1.0e-6f) {
            const float length = std::sqrt(lengthSquared);
            x /= length;
            y /= length;
            z /= length;
        }

        // Trig in double, narrowed once. Costs nothing here and keeps the rotation elements
        // within a rounding step of what QMatrix4x4 produced.
        const double radians = static_cast<double>(angleDegrees) * 3.14159265358979323846 / 180.0;
        const float c = static_cast<float>(std::cos(radians));
        const float s = static_cast<float>(std::sin(radians));
        const float ic = 1.0f - c;

        Mat4 rotation;
        rotation.set(0, 0, x * x * ic + c);
        rotation.set(0, 1, x * y * ic - z * s);
        rotation.set(0, 2, x * z * ic + y * s);
        rotation.set(1, 0, y * x * ic + z * s);
        rotation.set(1, 1, y * y * ic + c);
        rotation.set(1, 2, y * z * ic - x * s);
        rotation.set(2, 0, z * x * ic - y * s);
        rotation.set(2, 1, z * y * ic + x * s);
        rotation.set(2, 2, z * z * ic + c);
        *this = *this * rotation;
    }

    // Vertical field of view in degrees, as QMatrix4x4::perspective takes it.
    void perspective(float verticalAngleDegrees, float aspectRatio, float nearPlane, float farPlane) {
        if (nearPlane == farPlane || aspectRatio == 0.0f) return;

        const double radians =
            (static_cast<double>(verticalAngleDegrees) / 2.0) * 3.14159265358979323846 / 180.0;
        const double sine = std::sin(radians);
        if (sine == 0.0) return;
        const float cotan = static_cast<float>(std::cos(radians) / sine);
        const float clip = farPlane - nearPlane;

        Mat4 projection;
        projection.set(0, 0, cotan / aspectRatio);
        projection.set(1, 1, cotan);
        projection.set(2, 2, -(nearPlane + farPlane) / clip);
        projection.set(2, 3, -(2.0f * nearPlane * farPlane) / clip);
        projection.set(3, 2, -1.0f);
        projection.set(3, 3, 0.0f);
        *this = *this * projection;
    }

private:
    float m_data[16];  // column-major: m_data[column * 4 + row]
};
