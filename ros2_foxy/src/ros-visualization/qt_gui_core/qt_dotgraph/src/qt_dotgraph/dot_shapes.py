from python_qt_binding.QtCore import QRectF
from python_qt_binding.QtWidgets import QAbstractGraphicsShapeItem


class QGraphicsBox3dItem(QAbstractGraphicsShapeItem):

    def __init__(self, bounding_box):
        super(QGraphicsBox3dItem, self).__init__()
        self._bounding_box = bounding_box

    def boundingRect(self):
        return self._bounding_box

    def paint(self, painter, option, widget):
        # Main rectangle
        rectangle = QRectF(self._bounding_box.topLeft().x(),
                           self._bounding_box.topLeft().y() + self._bounding_box.height() * 0.1,
                           self._bounding_box.width() - self._bounding_box.height() * 0.1,
                           self._bounding_box.height() - self._bounding_box.height() * 0.1)
        painter.drawRect(rectangle)
        # Top line
        painter.drawLine(rectangle.topLeft().x() + self._bounding_box.height() * 0.1,
                         self._bounding_box.topLeft().y(),
                         self._bounding_box.topRight().x(),
                         self._bounding_box.topRight().y())
        # Top left corner
        painter.drawLine(rectangle.topLeft().x() + self._bounding_box.height() * 0.1,
                         self._bounding_box.topLeft().y(),
                         self._bounding_box.topLeft().x() + 1,
                         rectangle.topLeft().y())
        # Top right corner
        painter.drawLine(self._bounding_box.topRight().x(),
                         self._bounding_box.topRight().y(),
                         rectangle.topRight().x(),
                         rectangle.topRight().y())
        # Bottom right corner
        painter.drawLine(rectangle.bottomRight().x() + 1,
                         rectangle.bottomRight().y() - 1,
                         self._bounding_box.bottomRight().x(),
                         rectangle.bottomRight().y() - self._bounding_box.height() * 0.1)
        # Right line
        painter.drawLine(self._bounding_box.topRight().x(),
                         self._bounding_box.topRight().y(),
                         self._bounding_box.topRight().x(),
                         self._bounding_box.bottomRight().y() - self._bounding_box.height() * 0.1)
