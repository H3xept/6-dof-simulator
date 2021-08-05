#ifndef __MATRIXXD_EXTENSION_H__
#define __MATRIXXD_EXTENSION_H__

#include <Eigen/Eigen>

class MatrixXd : Eigen::MatrixXd {
    friend std::istream &operator>>(std::istream &is, MatrixXd& m) {
        for (int i = 0; i < m.rows(); i++) {
            for (int j = 0; j < m.cols(); j++) {
                is >> m(i,j);
            }
        }
        return is;
    }
};

#endif // __MATRIXXD_EXTENSION_H__