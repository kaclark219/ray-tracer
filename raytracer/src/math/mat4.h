#include <cmath>
#include <iostream>

class Mat4 {
    private:
        float data[4][4];

    public:
        // default constructor (identity matrix)
        Mat4() {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    data[i][j] = (i == j) ? 1.0f : 0.0f;
                }
            }
        }

        // filled matrix parameter constructor
        Mat4(float d[4][4]) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    data[i][j] = d[i][j];
                }
            }
        }

        // copy constructor
        Mat4(const Mat4 &m) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    data[i][j] = m.data[i][j];
                }
            }
        }

        // getters
        float get(int row, int col) const {
            return data[row][col];
        }

        // setters
        void set(int row, int col, float value) {
            data[row][col] = value;
        }

        // print matrix (for debugging)
        void print() const {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    std::cout << data[i][j] << " ";
                }
                std::cout << std::endl;
            }
        }
};