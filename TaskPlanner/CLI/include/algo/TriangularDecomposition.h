//
// Created by redwan on 3/22/21.
//

#ifndef BENCHMARKAREACOVERAGE_TRIANGULARDECOMPOSITION_H
#define BENCHMARKAREACOVERAGE_TRIANGULARDECOMPOSITION_H
#include <iostream>
#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
#include <armadillo>

using namespace std;

namespace TaskPlanning{
    class TriangularDecomposition {
    public:
        explicit TriangularDecomposition(const vector<double> &batteries);
        virtual std::vector<wykobi::polygon<double, 2>> solve(const wykobi::polygon<double,2>& target);
        int binary_search(const wykobi::polygon<double, 2>& search_poly, double x);

    protected:
        std::vector<double>m_get_capabilities(double area );
        wykobi::polygon<double, 2> m_get_search_space(const wykobi::polygon<double, 2>& roi);
    protected:
        std::vector<double> batteries_, capabilities_;

    };
}


#endif //BENCHMARKAREACOVERAGE_TRIANGULARDECOMPOSITION_H
