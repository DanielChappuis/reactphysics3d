
#include "mathematics.h"

#include <iostream>
#include <stdexcept>

using namespace std;

int main(int argc, char *argv[])
{
    Matrix matrix(3,3);
    matrix.setValue(0, 0, 4);
    matrix.setValue(0, 1, 64);
    matrix.setValue(0, 2, 6);

    matrix.setValue(1, 0, 73);
    matrix.setValue(1, 1, -64);
    matrix.setValue(1, 2, 5);

    matrix.setValue(2, 0, 3);
    matrix.setValue(2, 1, 976);
    matrix.setValue(2, 2, 70);

    matrix.getInverse().display();
}
