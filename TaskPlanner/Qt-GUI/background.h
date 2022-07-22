#ifndef BACKGROUND_H
#define BACKGROUND_H

#include <QObject>
#include <qqml.h>
#include <QDebug>
#include <iostream>
#include <vector>
#include <QVector>
#include "solver_interface.h"
#include <QThread>

class Background : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString mousePoint READ mousePoint WRITE setmousePoint NOTIFY mousePointChanged)
    Q_PROPERTY(QString cmdButton READ cmdButton WRITE setcmdButton NOTIFY cmdButtonChanged)
    Q_PROPERTY(QVector<double> lineX READ drawLineX WRITE setLineX NOTIFY drawLineXChanged)
    Q_PROPERTY(QVector<double> lineY READ drawLineY WRITE setLineY NOTIFY drawLineYChanged)
    Q_PROPERTY(QVector<double> robots READ drawRobots WRITE setRobots NOTIFY drawRobotsChanged)


public:
    explicit Background(QObject *parent = nullptr);

    virtual ~Background();

    QString cmdButton();
    void setcmdButton(const QString &cmdButton);

    QString mousePoint();
    void setmousePoint(const QString &mousePoint);

    QVector<double> drawLineX();
    QVector<double> drawLineY();
    QVector<double> drawRobots();



public slots:
    void setLineX(const QVector<double> &line);
    void setLineY(const QVector<double> &line);
    void setRobots(const QVector<double> &positions);


signals:
   void cmdButtonChanged();
   void mousePointChanged();
   void drawLineXChanged();
   void drawLineYChanged();
   void drawRobotsChanged();
private:
   enum ButtonState
   {
       DEFAULT,
       CLEAR,
       OKAY
   }m_cmdButton;
   std::vector<Point2D> m_mousePoints;
   QVector<double> m_lineX, m_lineY, m_robots;
   solver_interface *m_solver;
   pdfPtr m_pdf;
   QThread *m_cThread;
};

#endif // BACKGROUND_H
