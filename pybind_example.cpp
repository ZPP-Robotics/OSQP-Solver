#include <pybind11/pybind11.h>

int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(pybind_example, m) {
    m.doc() = "Test module";

    m.def("add", &add, "Add");
}

// Compiled with:
// c++ -O3 -shared -std=gnu++11 `python3-config --cflags --ldflags --libs` pybind_example.cpp -o pybind_example.so -fPIC -I/home/olaf/anaconda3/lib/python3.9/site-packages/pybind11/include
//
// Musi być skompilowane na dobrą wersję Pythona.
// Błąd:
// ImportError: /home/olaf/anaconda3/bin/../lib/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /home/olaf/Desktop/MIMUW/ZPP/OSQP-Solver/pybind_example.cpython-39-x86_64-linux-gnu.so)
//
// Chyba wynika z tego, że inna wersja GLIBCXX_3.4.29 jest w GCC i inna w Anacondzie.
// Rozwiązane przez:
// ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /home/olaf/anaconda3/bin/../lib/libstdc++.so.6
//
// Odpalane przez:
// python3.9 /home/olaf/Desktop/MIMUW/ZPP/OSQP-Solver/pybind_example.py