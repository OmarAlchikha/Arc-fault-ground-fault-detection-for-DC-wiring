"""
Export ADC-level test vectors + expected detector decisions so the C core
(firmware/arc_fault_detector/detector_core.h) can be verified bit-for-bit
against the Python reference on a PC.  Run test/run_host_test.sh to build
and execute the comparison.
"""

import os

import numpy as np

import waveforms as wf
import detector as det

OUT = os.path.join(os.path.dirname(__file__), "..", "test", "vectors")


def main():
    np.random.seed(42)
    os.makedirs(OUT, exist_ok=True)
    manifest = []
    idx = 0
    for gen in wf.ALL_GENERATORS:
        for rep in range(4):
            tr = gen()
            st, _ = det.process_trace(tr.hf_env, tr.i_lf)
            fname = f"{idx:03d}_{tr.name}.csv"
            data = np.column_stack([tr.hf_env, tr.i_lf]).astype(int)
            np.savetxt(os.path.join(OUT, fname), data, fmt="%d", delimiter=",",
                       header="hf_env,i_lf", comments="")
            manifest.append(f"{fname},{int(st.tripped)},{st.trip_window}")
            idx += 1
    with open(os.path.join(OUT, "manifest.csv"), "w") as f:
        f.write("file,expect_trip,expect_trip_window\n")
        f.write("\n".join(manifest) + "\n")
    print(f"wrote {idx} vectors to {os.path.abspath(OUT)}")


if __name__ == "__main__":
    main()
