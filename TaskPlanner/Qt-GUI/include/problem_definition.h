#ifndef PROBLEM_DEFINITION_H
#define PROBLEM_DEFINITION_H
#include <iostream>
#include <vector>
#include <memory>
#include<unordered_map>
#include "wykobi.hpp"
#include <QVector>

struct Point2D
{
    double x, y;
    Point2D operator * (double scale)
    {

        x = x * scale;
        y = y * scale;
        return *this;
    }

    Point2D operator / (double scale)
    {

        x = x / scale;
        y = y / scale;
        return *this;
    }

    /**
     * @brief operator -
     * @param other
     * @return compute Eucledean distance between two points
     */
    double operator - (const Point2D& other) const
    {
        double dist = sqrt( pow((x - other.x), 2) + pow((y - other.y), 2));
        return dist;
    }


    friend std::ostream & operator<<(std::ostream &os, const Point2D& p) {
        return os << p.x << ", " << p.y;
    }
};

class ProblemDefinition;
typedef std::shared_ptr<ProblemDefinition> pdfPtr;

class ProblemDefinition:public std::enable_shared_from_this<ProblemDefinition>{
public:
    using AGENT = std::pair<Point2D, double>;
    using POLY = wykobi::polygon<double, 2>;
    ProblemDefinition()
    {
        addAgent(Point2D{0.1, 0.1}, 5);
        addAgent(Point2D{0.9, 0.9}, 7);
        addAgent(Point2D{0.1, 0.9}, 5);
//        addAgent(Point2D{0.9, 0.1}, 7);
    }
    void addInstance(const std::vector<Point2D>& poly)
    {
        roi.clear();
        decomopsedROIs.clear();

        for(const Point2D& p: poly)
            roi.push_back(wykobi::make_point<double>(p.x, p.y));


    }

    QVector<double> flattenCoords()
    {
        QVector<double>res;
        for(auto agent:agents)
        {
            res.push_back(agent.first.x);
            res.push_back(agent.first.y);
        }
        return res;
    }

    pdfPtr getPtr()
    {
        return shared_from_this();
    }


    void addAgent(const Point2D& initPosition, double capacity)
    {
        agents.emplace_back(std::make_pair(initPosition, capacity));
    }

    const std::vector<double> capacities()
    {
        std::vector<double> res;
        for(const auto agent:agents)
            res.push_back(agent.second);
        return res;
    }

    void addDecomposedROI(const std::vector<Point2D>&roi)
    {

        decomopsedROIs.emplace_back(roi);
    }

    std::size_t size() const
    {
        return agents.size();
    }

    std::vector<Point2D> operator [](int index) const
    {

        return decomopsedROIs[index];
    }
    POLY roi;
    std::vector<AGENT> agents;
    std::vector<std::vector<Point2D>> decomopsedROIs;

};

#endif // PROBLEM_DEFINITION_H
