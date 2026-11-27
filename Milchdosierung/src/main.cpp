#include "Riverdi-RVT43/Riverdi-RVT43.h"

#include <cstdio>

int main()
{
    RiverdiRVT43 display;
    while (display.getState() == RiverdiRVT43::State::Initializing) {
        thread_sleep_for(10);
    }

    if (display.getState() == RiverdiRVT43::State::Ready && display.setText("MilkMate ready")) {
        while (display.getState() == RiverdiRVT43::State::Ready) {
            thread_sleep_for(100);
        }
    }

    std::printf("Display error\n");
    return 1;
}
