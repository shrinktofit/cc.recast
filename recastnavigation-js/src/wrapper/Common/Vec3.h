
#pragma once

namespace wasm_port {
struct Point3 {
    Point3() = default;

    Point3(float x_, float y_, float z_): x(x_), y(y_), z(z_) { }

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    float* data() {
        return &x;
    }

    const float* data() const {
        return &x;
    }
};
}