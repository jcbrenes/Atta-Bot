import os
import sys

# cv2 sobreescribe QT_QPA_PLATFORM_PLUGIN_PATH al importar, apuntando a sus
# propios plugins Qt (incompatibles con PyQt5).  Solución: importar cv2 primero,
# luego corregir la ruta y el backend antes de importar PyQt5.
import cv2

for _sp in sys.path:
    _candidate = os.path.join(_sp, 'PyQt5', 'Qt5', 'plugins')
    if os.path.isdir(_candidate):
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _candidate
        break

if os.environ.get('WAYLAND_DISPLAY'):
    os.environ['QT_QPA_PLATFORM'] = 'wayland'
else:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QComboBox, QFrame,
    QInputDialog, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QSize
from PyQt5.QtGui import QPixmap, QImage, QFont, QKeySequence


# ─────────────────────────────────────────────────────────────────────────────
# AttaBotGUI
# ─────────────────────────────────────────────────────────────────────────────

class AttaBotGUI(QMainWindow):
    """
    
    Ventana principal de control de AttaBot.

    Muestra la cámara (con anotaciones ArUco) y el mapa de cobertura
    en tiempo real, junto con un panel de comandos y un log de mensajes.

    Las actualizaciones de frame y log llegan desde hilos de fondo vía signals.
    """

    frameSignal = pyqtSignal(object, object)   # (camera_frame BGR, results_frame BGR)
    logSignal   = pyqtSignal(str)              # mensaje para el log

    def __init__(self, base):
        super().__init__()
        self.base = base
        self._cmdHistory  = []
        self._historyIdx  = -1
        self._buildUI()
        self.frameSignal.connect(self._onFrame)
        self.logSignal.connect(self._onLog)
        self.setWindowTitle('AttaBot Control')

    # ── construcción de la interfaz ──────────────────────────────────────────

    def _buildUI(self):
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.setSpacing(5)
        vbox.setContentsMargins(6, 6, 6, 6)

        # ── fila de cámara ───────────────────────────────────────────────────
        camRow = QHBoxLayout()
        camRow.setSpacing(5)

        self._camLabel = self._makeFrameLabel('Cámara')
        self._mapLabel = self._makeFrameLabel('Mapa de cobertura')
        camRow.addWidget(self._camLabel, stretch=1)
        camRow.addWidget(self._mapLabel, stretch=1)
        vbox.addLayout(camRow, stretch=1)

        # ── panel de comandos ────────────────────────────────────────────────
        cmdFrame = QFrame()
        cmdFrame.setFrameShape(QFrame.StyledPanel)
        cmdBox = QVBoxLayout(cmdFrame)
        cmdBox.setSpacing(4)
        cmdBox.setContentsMargins(6, 4, 6, 4)

        # Fila 1: selector de robot + campo de comando + botón enviar
        row1 = QHBoxLayout()
        row1.setSpacing(6)

        row1.addWidget(QLabel('Robot:'))

        self._robotCombo = QComboBox()
        self._robotCombo.setMinimumWidth(110)
        self._robotCombo.addItem('BROADCAST')
        row1.addWidget(self._robotCombo)

        self._cmdInput = QLineEdit()
        self._cmdInput.setPlaceholderText('Comando (ej: GT|500|300 — Enter para enviar)')
        self._cmdInput.returnPressed.connect(self._send)
        self._cmdInput.installEventFilter(self)
        row1.addWidget(self._cmdInput, stretch=1)

        sendBtn = QPushButton('Enviar ↵')
        sendBtn.setFixedWidth(80)
        sendBtn.clicked.connect(self._send)
        row1.addWidget(sendBtn)
        cmdBox.addLayout(row1)

        # Fila 2: movimiento
        row2 = QHBoxLayout()
        row2.setSpacing(4)
        movButtons = [
            ('MOVE',      'MOVE|',              False),
            ('TURN',      'TURN|',              False),
            ('RANDOMW',   'RANDOMW|',           False),
            ('WAIT',      'WAIT|',              False),
            ('GT',        'GT|',               False),
            ('ABORT_NAV', 'ABORT_NAV',          True),
            ('CLEAR_EV',  'CLEAR_EVASION',      True),
            ('RST_EV',    'RESET_EVASION',      True),
        ]
        for label, cmd, direct in movButtons:
            btn = QPushButton(label)
            btn.setMaximumWidth(88)
            if direct:
                btn.clicked.connect(lambda _, c=cmd: self._quickSend(c))
            else:
                btn.clicked.connect(lambda _, c=cmd: self._fillInput(c))
            row2.addWidget(btn)
        row2.addStretch()
        cmdBox.addLayout(row2)

        # Fila 3: sistema / config
        row3 = QHBoxLayout()
        row3.setSpacing(4)
        sysButtons = [
            ('STATUS',    'GET_STATUS',          True),
            ('GET_YAW',   'GET_YAW',             True),
            ('GETPPR',    'GETPPR',              True),
            ('PID',       'PID|',               False),
            ('CONGR',     'CONGREGATION|',      False),
            ('CANCEL_C',  'CANCEL_CONGREGATION', True),
            ('RESET',     'RESET',               True),
            ('ORIGIN [o]','__ORIGIN__',          True),
        ]
        for label, cmd, direct in sysButtons:
            btn = QPushButton(label)
            btn.setMaximumWidth(88)
            if direct:
                btn.clicked.connect(lambda _, c=cmd: self._quickSend(c))
            else:
                btn.clicked.connect(lambda _, c=cmd: self._fillInput(c))
            row3.addWidget(btn)
        row3.addStretch()
        cmdBox.addLayout(row3)

        vbox.addWidget(cmdFrame)

        # ── log ──────────────────────────────────────────────────────────────
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFixedHeight(160)
        self._log.setFont(QFont('Monospace', 9))
        vbox.addWidget(self._log)

    @staticmethod
    def _makeFrameLabel(placeholder: str) -> QLabel:
        lbl = QLabel(placeholder)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setMinimumSize(QSize(560, 315))
        lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lbl.setStyleSheet('background:#111; color:#555; border:1px solid #333;')
        return lbl

    # ── actualización de robots ──────────────────────────────────────────────

    def refreshRobots(self):
        """Actualiza el dropdown con los robots actualmente registrados."""
        prev = self._robotCombo.currentText()
        self._robotCombo.clear()
        self._robotCombo.addItem('BROADCAST')
        for rid, r in sorted(self.base.robots.items()):
            self._robotCombo.addItem(f'{rid} ({r.name})', userData=rid)
        idx = self._robotCombo.findText(prev)
        if idx >= 0:
            self._robotCombo.setCurrentIndex(idx)

    # ── envío de comandos ────────────────────────────────────────────────────

    def _selectedRobotId(self) -> str:
        data = self._robotCombo.currentData()
        return data if data is not None else 'BROADCAST'

    def _send(self):
        cmd = self._cmdInput.text().strip()
        if not cmd:
            return
        robot_id = self._selectedRobotId()
        self._dispatch(robot_id, cmd)
        if not self._cmdHistory or self._cmdHistory[0] != cmd:
            self._cmdHistory.insert(0, cmd)
        self._historyIdx = -1
        self._cmdInput.clear()

    def _quickSend(self, cmd: str):
        if cmd == '__ORIGIN__':
            self.base.recalibrateOrigin()
            self.logSignal.emit('[ORIGIN] Origen recalibrado')
            return
        self._fillInput(cmd)
        self._send()

    def _fillInput(self, text: str):
        self._cmdInput.setText(text)
        self._cmdInput.setFocus()
        self._cmdInput.setCursorPosition(len(text))

    def _dispatch(self, robot_id: str, instruction: str):
        """Mismo comportamiento que inputInstruction() pero desde la GUI."""
        label = f'{robot_id}.{instruction}'
        self.logSignal.emit(f'<span style="color:#7af;"><b>&gt;&gt; {label}</b></span>')

        if robot_id == 'BROADCAST':
            self.base.sendInstructionBroadcast([instruction])

        elif robot_id == 'CONGREGATION':
            self.base.startCongregation(instruction)

        elif robot_id == 'GOTO':
            # Formato: GOTO.robotID x y
            parts = instruction.split()
            if len(parts) == 3:
                try:
                    self.base.sendToGlobalPosition(parts[0], float(parts[1]), float(parts[2]))
                except ValueError:
                    self.logSignal.emit('GOTO: formato inválido — use robotID x y')
            else:
                self.logSignal.emit('GOTO: formato inválido — use robotID x y')

        elif robot_id == 'STATUS':
            for rid, robot in self.base.robots.items():
                x, y, angle = robot.getPose()
                if x != -1:
                    self.logSignal.emit(f'  {robot.name}: ({x:.1f},{y:.1f}) {angle:.1f}°')
                else:
                    self.logSignal.emit(f'  {robot.name}: no visible')

        elif robot_id in self.base.robots:
            ip = self.base.robots[robot_id].IP
            self.base.sendInstruction(ip, [instruction], False)

        else:
            self.logSignal.emit(f'Robot "{robot_id}" no encontrado')

    # ── historial de comandos (↑ ↓) ──────────────────────────────────────────

    def eventFilter(self, obj, event):
        from PyQt5.QtCore import QEvent
        from PyQt5.QtGui import QKeyEvent
        if obj is self._cmdInput and event.type() == QEvent.KeyPress:
            key = event.key()
            if key == Qt.Key_Up and self._cmdHistory:
                self._historyIdx = min(self._historyIdx + 1, len(self._cmdHistory) - 1)
                self._cmdInput.setText(self._cmdHistory[self._historyIdx])
                return True
            if key == Qt.Key_Down:
                self._historyIdx = max(self._historyIdx - 1, -1)
                self._cmdInput.setText(
                    self._cmdHistory[self._historyIdx] if self._historyIdx >= 0 else ''
                )
                return True
        return super().eventFilter(obj, event)

    # ── slots de signals ─────────────────────────────────────────────────────

    @pyqtSlot(object, object)
    def _onFrame(self, cam_frame, results_frame):
        if cam_frame is not None:
            self._camLabel.setPixmap(
                self._npToPixmap(cam_frame, self._camLabel.size())
            )
        if results_frame is not None:
            self._mapLabel.setPixmap(
                self._npToPixmap(results_frame, self._mapLabel.size())
            )

    @pyqtSlot(str)
    def _onLog(self, msg: str):
        self._log.append(msg)
        self._log.verticalScrollBar().setValue(
            self._log.verticalScrollBar().maximum()
        )

    # ── conversión numpy → QPixmap ────────────────────────────────────────────

    @staticmethod
    def _npToPixmap(frame: np.ndarray, target: QSize) -> QPixmap:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.tobytes(), w, h, ch * w, QImage.Format_RGB888)
        return QPixmap.fromImage(qimg).scaled(
            target, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )


# ─────────────────────────────────────────────────────────────────────────────
# Punto de entrada con GUI
# ─────────────────────────────────────────────────────────────────────────────

def launch(base_instance):
    """
    Arranca la aplicación PyQt5 para AttaBot.

    - Pregunta la cantidad de robots con un diálogo.
    - Inicia el procesamiento de cámara en hilo de fondo.
    - Corre el event loop de Qt en el hilo principal.
    """
    import threading

    app = QApplication.instance() or QApplication(sys.argv)

    numRobots, ok = QInputDialog.getInt(
        None, 'AttaBot', 'Cantidad de robots en la prueba:', 1, 1, 10
    )
    if not ok:
        return

    base_instance.numRobots = numRobots
    base_instance.readConfigFile('configSystem.json')

    gui = AttaBotGUI(base_instance)
    base_instance.gui = gui
    gui.show()

    cam_thread = threading.Thread(target=base_instance.cameraProcessing, daemon=True)
    cam_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    import sys
    import os
    sys.path.insert(0, os.path.dirname(__file__))
    from AttaBot_Base import base
    launch(base)
