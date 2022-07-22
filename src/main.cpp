#include "rigid2d/simulator.h"


using namespace rigid2d;

int main(int argc, char* argv[]) {
    Simulator app(1280, 720, "test-scene.json");
    app.Run();
    return 0;
}
