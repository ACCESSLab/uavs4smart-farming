import QtQuick 2.11
import QtQuick.Window 2.11
import QtQuick.Controls 2.12
import BackEnd 0.1


ApplicationWindow {
    id: mainWindow
    width: 1280
    height: 720
    visible: true
    title: qsTr("Area Planner")
    Rectangle{
        id: rectangle
        width: parent.width
        height: parent.height
        color: 'darkgray'


        BackEnd{
            id: backend
        }

        Canvas {
                id: myCanvas
                anchors.rightMargin: 0
                anchors.bottomMargin: 0
                anchors.leftMargin: 0
                anchors.topMargin: 0
                anchors.fill: parent
                renderTarget: Canvas.Image
                renderStrategy: Canvas.Immediate
                onPaint: {


                    if(mouseClicked.leftClicked)
                    {
                        var ctx = getContext('2d')
                        ctx.globalCompositeOperation = "source-over";
                        ctx.fillStyle = "white"
//                        https://stackoverflow.com/questions/26044801/draw-an-arc-circle-sector-in-qml
                        ctx.arc(mouseClicked.mouseX, mouseClicked.mouseY, 10, Math.PI * 0.5, Math.PI * 6, true)
                        ctx.lineTo(mouseClicked.mouseX, mouseClicked.mouseY);
                        ctx.fill();
                    }


                }
                function clear_canvas()
                {
                    var ctx = getContext("2d");
                    ctx.reset();
                    mouseClicked.leftClicked = false;
                    myCanvas.requestPaint();
               }

               function draw_polygon()
               {
                   const lineX = backend.lineX;
                   const lineY = backend.lineY;
                   if(lineX.length + lineY.length <= 0)
                       return
                   if(lineX.length !== lineY.length )
                       return

//                   myCanvas.clear_canvas();
                   // get context to draw with
                  var ctx = getContext("2d")
                  // setup the stroke
                  ctx.lineWidth = 10
                  ctx.strokeStyle = "darkgray"

                  // setup the fill
                  ctx.fillStyle = "steelblue"
                  // begin a new path to draw
                  ctx.beginPath()
                  // top-left start point


                   for (let i = 1; i < lineX.length; i++) {
                     ctx.moveTo(lineX[i-1] * rectangle.width, lineY[i-1] * rectangle.height)
                     ctx.lineTo(lineX[i] * rectangle.width,lineY[i] * rectangle.height)
                   }
                   ctx.moveTo(lineX[lineX.length-1] * rectangle.width, lineY[lineX.length-1] * rectangle.height)
                   ctx.lineTo(lineX[0] * rectangle.width,lineY[0] * rectangle.height)


                  // left line through path closing
                  ctx.closePath()
                  // fill using fill style
                  ctx.fill()
                  // stroke using line width and stroke style
                  ctx.stroke();
               }

        }
        MouseArea {
            id: mouseClicked
            width: parent.width
            height: parent.height
            anchors.fill: parent

            property double xx
            property double yy
            property bool leftClicked: false
            onClicked: {

                xx = mouse.x/rectangle.width;
                yy = mouse.y/rectangle.height;
                leftClicked = true;
                backend.mousePoint = xx.toString() + "," + yy.toString();
//                console.log("mouse button pressed ", xx, yy);
                myCanvas.requestPaint();
            }

        }

        Button
        {
            id: cls
            text: "Clear"
            x: parent.width - width
            background:
                Rectangle {
                        color: parent.down ? "#bbbbbb" :
                            (parent.hovered ? "#d6d6d6" : "#f6f6f6")
                    }
            onClicked:
            {
//                console.log("clear canvas");

                backend.cmdButton = "clear";
                mouseClicked.leftClicked = false;
                myCanvas.clear_canvas();
            }

        }

        Button
        {
            id: ok
            text: "Okay"
            y: cls.height + height/2
            x: parent.width - width
            background:
                Rectangle {
                        color: parent.down ? "#bbbbbb" :
                            (parent.hovered ? "#d6d6d6" : "#f6f6f6")
                    }
            onClicked:
            {
//                console.log("okay button pressed");
                backend.cmdButton = "okay";


            }
        }

        Rectangle
        {
            id:robotA
            width: 20
            height: 20
            color: "red"
            x: rectangle.width * backend.robots[0]
            y: rectangle.height * backend.robots[1]
        }

        Rectangle
        {
            id:robotB
            width: 20
            height: 20
            color: "cyan"
            x: rectangle.width * backend.robots[2]
            y: rectangle.height * backend.robots[3]
        }

        Rectangle
        {
            id:robotC
            width: 20
            height: 20
            color: "magenta"
            x: rectangle.width * backend.robots[4]
            y: rectangle.height * backend.robots[5]
            onXChanged:
            {
                myCanvas.draw_polygon();
                myCanvas.requestPaint();
            }
        }

//        Rectangle
//        {
//            id:robotD
//            width: 20
//            height: 20
//            color: "blue"
//            x: rectangle.width * backend.robots[6]
//            y: rectangle.height * backend.robots[7]

//        }


    }
}
