#include <iostream>
#include "rigid2d/geometry.h"
#include "rigid2d/simulator.h"


using namespace rigid2d;

int main() {
    Simulator app(500, 500);
    auto mesh = std::make_shared<TriangleMesh>("test", "test.obj");
    app.Run(mesh);
    return 0;
}
