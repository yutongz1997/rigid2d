#include "rigid2d/simulator.h"


using namespace rigid2d;

int main(int argc, char* argv[]) {
    Simulator app(800, 600, "simple.json");
    app.Run();
    return 0;
}
