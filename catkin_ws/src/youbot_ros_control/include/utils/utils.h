#ifndef UTILS_H
#define UTILS_H

// return variable sign either 1, 0 or -1
template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}


#endif
