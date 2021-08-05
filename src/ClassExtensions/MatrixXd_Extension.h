#ifndef __MATRIXXD_EXTENSION_H__
#define __MATRIXXD_EXTENSION_H__

#include <Eigen/Eigen>

/**
 * 
 * Eigen::MatrixXd extension to allow population by stream.
 * The values for the matrix must be newline-separated.
 * E.g (2x2): 
 * 
 * |0 1|
 * |2 3|
 * 
 * vvvvv
 * 
 * - 0
 * - 1
 * - 2
 * - 3
 * 
 */
struct MatrixXd : public Eigen::MatrixXd {
    friend std::istream &operator>>(std::istream &is, MatrixXd& m) {
        for (int i = 0; i < m.rows(); i++) {
            for (int j = 0; j < m.cols(); j++) {
                printf("Ma\n");
                is >> m(i,j);
            }
        }
        return is;
    }
};

#endif // __MATRIXXD_EXTENSION_H__