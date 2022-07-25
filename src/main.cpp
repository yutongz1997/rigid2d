#include "rigid2d/simulator.h"


using namespace rigid2d;

int main(int argc, char* argv[]) {
    Simulator app(800, 800, "test-scene.json");
    app.Run();
    return 0;
}
