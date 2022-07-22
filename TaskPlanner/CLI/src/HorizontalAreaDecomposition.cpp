//
// Created by robor on 6/20/2020.
//

#include "algo/HorizontalAreaDecomposition.h"
using namespace TaskPlanning;
#define MAX_RANGE (100)
#define MIN_RANGE (0)
HorizontalAreaDecomposition::HorizontalAreaDecomposition(const vector<double> &batteries) : MinTurnDecomposition(
        batteries) {}

vector<wykobi::polygon<double, 2>> HorizontalAreaDecomposition::solve(const wykobi::polygon<double, 2> &target) {
    // matrix representation of a rectangle
    /*
     *      D------C
     *      |      |
     *      A------B
     */
    arma::mat22 AD, BC, EF, REF;
    AD(0,0) = MIN_RANGE;    // Ay
    AD(1,0) = MAX_RANGE;    // Dy
    AD(1,1) = MIN_RANGE;    // Dx
    AD(0,1) = MIN_RANGE;    // Ax

    BC(0,0) = MIN_RANGE;    // By
    BC(1,0) = MAX_RANGE;    // Cy
    BC(1,1) = MAX_RANGE;    // Cx
    BC(0,1) = MAX_RANGE;    // Bx

    std::vector<wykobi::polygon<double, 2>>result;
    capabilities_ = m_get_capabilities(abs(wykobi::area(target)));
    m_decompose_area(target, AD, BC, EF, REF, result);
    return result;

}
