// Host-side parity test: runs the EXACT algorithm the ATmega2560 runs
// (firmware/arc_fault_detector/detector_core.h) over ADC traces exported
// by sim/export_test_vectors.py and checks that trip decision AND trip
// window match the Python reference bit-for-bit.
//
// Build/run:  test/run_host_test.sh

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include "../firmware/arc_fault_detector/detector_core.h"

int main(int argc, char **argv) {
    std::string dir = (argc > 1) ? argv[1] : "test/vectors";
    std::ifstream mf(dir + "/manifest.csv");
    if (!mf) { std::fprintf(stderr, "cannot open %s/manifest.csv "
                            "(run sim/export_test_vectors.py first)\n", dir.c_str());
               return 2; }

    std::string line;
    std::getline(mf, line);  // header
    int total = 0, failures = 0;

    while (std::getline(mf, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string fname, s_trip, s_win;
        std::getline(ss, fname, ',');
        std::getline(ss, s_trip, ',');
        std::getline(ss, s_win, ',');
        int expect_trip = std::atoi(s_trip.c_str());
        int expect_win  = std::atoi(s_win.c_str());

        std::ifstream df(dir + "/" + fname);
        if (!df) { std::fprintf(stderr, "missing %s\n", fname.c_str()); return 2; }
        std::string dl;
        std::getline(df, dl);  // header
        std::vector<uint16_t> hf, lf;
        while (std::getline(df, dl)) {
            unsigned a, b;
            if (std::sscanf(dl.c_str(), "%u,%u", &a, &b) == 2) {
                hf.push_back((uint16_t)a);
                lf.push_back((uint16_t)b);
            }
        }

        af_state_t st;
        af_init(&st);
        int trip_win = -1;
        size_t n_win = hf.size() / AF_WIN;
        for (size_t w = 0; w < n_win; w++) {
            uint8_t r = af_process_window(&st, &hf[w * AF_WIN], &lf[w * AF_WIN]);
            if (r != AF_NO_TRIP && trip_win < 0) trip_win = (int)w;
        }

        total++;
        bool ok = ((int)st.tripped == expect_trip) &&
                  (expect_trip == 0 || trip_win == expect_win);
        if (!ok) {
            failures++;
            std::printf("FAIL %-32s  expected trip=%d win=%d, got trip=%d win=%d\n",
                        fname.c_str(), expect_trip, expect_win, st.tripped, trip_win);
        }
    }

    std::printf("%d/%d vectors match the Python reference\n",
                total - failures, total);
    return failures ? 1 : 0;
}
