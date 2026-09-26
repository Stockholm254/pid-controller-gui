"""
main.py - Main script


MainApplication
    customized QApplication which encapsulates settings, controller remote connection

MainWindow
    QMainWindow subclass for tools and status bars, some actions and signals/slots

CentralWidget
    remaining UI elements such as PID values GroupBox'es, live graphs
"""

import multiprocessing
import sys

from PyQt5.QtCore import Qt, QCoreApplication, QTimer, pyqtSlot, pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QGridLayout, QHBoxLayout, QLabel, QAction, QComboBox
from PyQt5.QtGui import QIcon

import qdarkstyle

# local imports
import util
import remotecontroller
import miscgraphics
import graphs
import settings
import errorssettings
import about
from pathlib import Path



DERIVATIVE_GRAPH_NAME = "dV/dt"  # smoothed derivative of the process variable the controller's D term acts on
DERIVATIVE_GRAPH_INDEX = 1  # position in the stream point (process variable, dV/dt, controller output)

# names shown by the mode switch, keys match remotecontroller.mode
MODE_NAMES = {
    'pid': "PID",
    'damp': "Derivative damp"
}
# ValueGroupBox'es the controller uses in each mode, all others are grayed out
MODE_CONTROLS = {
    'pid': ('setpoint', 'kP', 'kI', 'kD', 'd_window'),
    'damp': ('kD', 'd_window')
}



class CentralWidget(QWidget):
    """
    CentralWidget holds ValueGroupBox'es corresponding to PID setpoint and coefficients, live graphs and averaging
    labels
    """

    def __init__(self, app: QApplication, parent=None):
        """
        CentralWidget constructor

        :param app: parent MainApplication
        :param parent: [optional] parent class
        """

        super(CentralWidget, self).__init__(parent)

        self.app = app

        grid = QGridLayout()
        self.setLayout(grid)


        self.contValGroupBoxes = [
            miscgraphics.ValueGroupBox('setpoint', float_fmt=app.settings['pid']['valueFormat'], conn=app.conn),
            miscgraphics.ValueGroupBox('kP', float_fmt=app.settings['pid']['valueFormat'], conn=app.conn),
            miscgraphics.ValueGroupBox('kI', float_fmt=app.settings['pid']['valueFormat'], conn=app.conn),
            miscgraphics.ValueGroupBox('kD', float_fmt=app.settings['pid']['valueFormat'], conn=app.conn),
            miscgraphics.ValueGroupBox('d_window', float_fmt=app.settings['pid']['valueFormat'], conn=app.conn,
                                       displayName="D window (s)")
        ]

        for groupBox, yPosition in zip(self.contValGroupBoxes, [0,3,6,9,12]):
            grid.addWidget(groupBox, yPosition, 0, 3, 2)


        self.graphs = graphs.CustomGraphicsLayoutWidget(
            names=(app.settings['pid']['processVariable']['name'],
                   DERIVATIVE_GRAPH_NAME,
                   app.settings['pid']['controllerOutput']['name']),
            numPoints=app.settings['graphs']['numberOfPoints'],
            interval=app.settings['graphs']['updateInterval'],
            ranges=((app.settings['pid']['processVariable']['limits']['min'],
                     app.settings['pid']['processVariable']['limits']['max']),
                    None,  # auto-range
                    (app.settings['pid']['controllerOutput']['limits']['min'],
                     app.settings['pid']['controllerOutput']['limits']['max'])),
            units=(app.settings['pid']['processVariable']['unit'],
                   app.settings['pid']['processVariable']['unit'] + "/s",
                   app.settings['pid']['controllerOutput']['unit']),
            controlPipe=None if app.isOfflineMode else app.conn.input_thread_control_pipe_main,
            streamPipeRX=None if app.isOfflineMode else app.conn.stream.pipe_rx,
            theme=app.settings['appearance']['theme'],
        )

        for averageLabel, name, yPosition in zip(self.graphs.averageLabels, self.graphs.names, [15,16,17]):
            hBox = QHBoxLayout()
            hBox.addWidget(QLabel(name))
            hBox.addWidget(averageLabel, alignment=Qt.AlignLeft)
            grid.addLayout(hBox, yPosition, 0, 1, 2)

        grid.addWidget(self.graphs, 0, 2, 18, 6)


    def setMode(self, mode: str) -> None:
        """
        Adapt the UI to the controller mode: gray out the values the mode doesn't use and show the dV/dt graph in the
        derivative damp mode only

        :param mode: key of remotecontroller.mode
        :return: None
        """

        for groupBox in self.contValGroupBoxes:
            groupBox.setEnabled(groupBox.label in MODE_CONTROLS[mode])
        self.graphs.setGraphVisible(DERIVATIVE_GRAPH_INDEX, mode == 'damp')


    def updateDisplayingValues(self) -> None:
        """
        Retrieve all controller parameters and update corresponding GUI elements. Useful to apply after connection's
        breaks. This does not affect values saved during the app launch (RemoteController.save_current_values())

        :return: None
        """

        for groupBox in self.contValGroupBoxes:
            groupBox.refreshVal()

        self.app.mainWindow.errorsSettingsWindow.updateDisplayingValues('err_P_limits', 'err_I_limits')
        self.app.mainWindow.refreshMode()




class MainWindow(QMainWindow):
    """
    MainWindow contains of toolbar, status bar, menu. All other items are placed on a CentralWidget
    """

    def __init__(self, app: QApplication, parent=None):
        """
        MainWindow constructor

        :param app: parent MainApplication
        :param parent: [optional] parent class
        """

        super(MainWindow, self).__init__(parent)

        self.app = app

        self.setWindowTitle(QCoreApplication.applicationName())
        self.setWindowIcon(QIcon(util.resource_path('../img/icon.png')))


        #
        # App Toolbar section
        #
        exitAction = QAction(QIcon(util.resource_path('../img/exit.png')), 'Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip("[Ctrl+Q] Exit application")
        exitAction.triggered.connect(self.app.quit)

        aboutAction = QAction(QIcon(util.resource_path('../img/info.png')), 'About', self)  # see about.py
        aboutAction.setShortcut('Ctrl+I')
        aboutAction.setStatusTip("[Ctrl+I] Application Info & About")
        self.aboutWindow = about.AboutWindow()
        aboutAction.triggered.connect(self.aboutWindow.show)

        settingsAction = QAction(QIcon(util.resource_path('../img/settings.png')), 'Settings', self)  # see settings.py
        settingsAction.setShortcut('Ctrl+P')
        settingsAction.setStatusTip("[Ctrl+P] Application Settings")
        self.settingsWindow = settings.SettingsWindow(app=app)
        settingsAction.triggered.connect(self.settingsWindow.show)

        appToolbar = self.addToolBar('app')  # internal name
        appToolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        appToolbar.addAction(exitAction)
        appToolbar.addAction(aboutAction)
        appToolbar.addAction(settingsAction)

        #
        # Controller Toolbar section
        #
        # see errorssettings.py
        errorsSettingsAction = QAction(QIcon(util.resource_path('../img/set_errors.png')), 'Errors limits', self)
        errorsSettingsAction.setShortcut('E')
        errorsSettingsAction.setStatusTip("[E] Set values of errors limits")
        self.errorsSettingsWindow = errorssettings.ErrorsSettingsWindow(app=app)
        errorsSettingsAction.triggered.connect(self.errorsSettingsWindow.show)

        restoreValuesAction = QAction(QIcon(util.resource_path('../img/restore.png')), 'Restore controller', self)
        restoreValuesAction.setShortcut('R')
        restoreValuesAction.setStatusTip("[R] Restore all controller parameters to values at the program start time")
        restoreValuesAction.triggered.connect(self.restoreContValues)

        saveToEEPROMAction = QAction(QIcon(util.resource_path('../img/eeprom.png')), 'Save to EEPROM', self)
        saveToEEPROMAction.setShortcut('S')
        saveToEEPROMAction.setStatusTip("[S] Save current controller configuration to EEPROM")
        saveToEEPROMAction.triggered.connect(self.saveToEEPROM)

        contToolbar = self.addToolBar('controller')  # internal name
        contToolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        contToolbar.addAction(errorsSettingsAction)
        contToolbar.addAction(restoreValuesAction)
        contToolbar.addAction(saveToEEPROMAction)

        #
        # Graphs Toolbar section
        #
        lockAction = QAction(QIcon(util.resource_path('../img/play_pause.png')), 'Lock/Unlock', self)
        lockAction.setShortcut('L')
        lockAction.setStatusTip("[L] Lock/Unlock PID")
        lockAction.triggered.connect(self.lockPID)

        self.modeComboBox = QComboBox()
        self.modeComboBox.addItems(MODE_NAMES.values())
        self.modeComboBox.setStatusTip("Controller mode. PID: lock on the setpoint. Derivative damp: only kD * dV/dt "
                                       "around the output at the time of switching/locking (setpoint, kP, kI unused)")

        playpauseAction = QAction(QIcon(util.resource_path('../img/play_pause.png')), 'Play/Pause', self)
        playpauseAction.setShortcut('P')
        playpauseAction.setStatusTip("[P] Play/pause graphs")
        playpauseAction.triggered.connect(self.playpauseGraphs)

        graphsToolbar = self.addToolBar('graphs')  # internal name
        graphsToolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        graphsToolbar.addAction(lockAction)
        graphsToolbar.addWidget(QLabel(" Mode: "))
        graphsToolbar.addWidget(self.modeComboBox)
        graphsToolbar.addAction(playpauseAction)
        self.playpauseButton = graphsToolbar.widgetForAction(playpauseAction)
        self.playpauseButton.setCheckable(True)
        self.playpauseButton.setChecked(True)
        self.graphsWereRun = False

        self.lockButton = graphsToolbar.widgetForAction(lockAction)
        self.lockButton.setCheckable(True)
        self.lockButton.setChecked(False)

        mainMenu = self.menuBar().addMenu('&Menu')
        mainMenu.addAction(aboutAction)
        mainMenu.addAction(settingsAction)
        mainMenu.addAction(exitAction)


        if self.app.isOfflineMode:
            self.statusBar().addWidget(QLabel("<font color='red'>Offline mode</font>"))

        self.centralWidget = CentralWidget(app=app)
        self.setCentralWidget(self.centralWidget)

        # the mode switch adapts the CentralWidget so it can be used only now
        self.refreshMode()
        self.modeComboBox.currentIndexChanged.connect(self.modeChanged)

        self.statusBar().show()  # can be not visible in online mode otherwise


    def playpauseGraphs(self) -> None:
        """
        Smartly toggles the state of live graphs

        :return: None
        """

        if self.centralWidget.graphs.isRun:
            self.playpauseButton.setChecked(True)
        else:
            self.playpauseButton.setChecked(False)
        self.app.conn.stream.toggle()
        self.centralWidget.graphs.toggle()

    def lockPID(self) -> None:
        """
        Smartly toggles the state of lock

        :return: None
        """
        
        if self.lockButton.isChecked():
            print("stop lock")
            if self.app.conn.stop_lock() == remotecontroller.result['ok']:
                self.lockButton.setChecked(False)
        else:
            print("start lock")
            if self.app.conn.start_lock() == remotecontroller.result['ok']:
                self.lockButton.setChecked(True)


    def _showMode(self, mode: str) -> None:
        """
        Set the mode switch (without sending anything to the controller) and adapt the rest of the UI

        :param mode: key of remotecontroller.mode
        :return: None
        """

        self.mode = mode
        self.modeComboBox.blockSignals(True)
        self.modeComboBox.setCurrentIndex(list(MODE_NAMES).index(mode))
        self.modeComboBox.blockSignals(False)
        self.centralWidget.setMode(mode)


    def refreshMode(self) -> None:
        """
        Read the mode from the controller and show it (PID in the offline mode)

        :return: None
        """

        mode = 'pid'
        if not self.app.isOfflineMode and self.app.conn.read('mode') == remotecontroller.mode['damp']:
            mode = 'damp'
        self._showMode(mode)


    def modeChanged(self, index: int) -> None:
        """
        Send the mode picked by the mode switch to the controller (callback for modeComboBox.currentIndexChanged slot).
        If the controller is locked it switches over without a jump of the output

        :param index: index of the picked item
        :return: None
        """

        mode = list(MODE_NAMES)[index]
        print(f"mode: {mode}")
        # an exception escaping a Qt slot would abort the whole app
        try:
            ok = self.app.conn.write('mode', remotecontroller.mode[mode]) == remotecontroller.result['ok']
        except (remotecontroller.ResponseException, remotecontroller.ResponseVarCmdMismatchException,
                remotecontroller.ResponseOperationMismatchException) as e:
            miscgraphics.MessageWindow(f"Mode switch failed: {e}", status='Error')
            ok = False

        if ok:
            self._showMode(mode)
            self.statusBar().showMessage(f"Mode: {MODE_NAMES[mode]}")
        else:
            self._showMode(self.mode)  # revert the switch


    def restoreContValues(self) -> None:
        """
        Write PID parameters stored at program start to the controller

        :return: None
        """

        date =self.app.conn.restore_values(self.app.conn.snapshots[0])  # currently save and use only one snapshot
        print(f"Snapshot from {date} is restored")
        self.centralWidget.updateDisplayingValues()


    def saveToEEPROM(self) -> None:
        """
        Initiate writing of current parameters to the controller's EEPROM

        :return: None
        """

        if self.app.conn.save_to_eeprom() == remotecontroller.result['ok']:
            miscgraphics.MessageWindow("Successfully saved", status='Info')
            self.centralWidget.updateDisplayingValues()
        else:
            miscgraphics.MessageWindow("Saving failed!", status='Error')


    def hideEvent(self, *args, **kwargs) -> None:
        """
        Detect window hide event to perform necessary actions. Modern OSes put minimized DE applications into some kind
        of 'sleep' mode to optimize its own performance and energy consumption. This leads to, for example, QT timers to
        be slow down and so on. And it may lead to some violations in a work flow such as connection breaks and stream
        overflows. So it better to 'freeze' any network and graphical activity during such period and then smoothly
        resume it when needed. The user doesn't stare at the application, anyway :)

        Note: this doesn't include simple moving a window to the background (e.g. overlapping by another window)

        :param args: positional arguments passing directly to the base-class function
        :param kwargs: keyword arguments passing directly to the base-class function
        :return: None
        """

        # 1. stop the connection check timer
        if not self.app.isOfflineMode:
            self.app.connCheckTimer.stop()

        # 2. stop live plots (it stops both the stream and graphs)
        if self.centralWidget.graphs.isRun:
            self.playpauseGraphs()
            self.graphsWereRun = True
        else:
            self.graphsWereRun = False

        # 3. in the end block the listening section of the input thread (to prevent pipes overflows)
        self.app.conn.pause()

        # finally, call the base class' method
        super(MainWindow, self).hideEvent(*args, **kwargs)


    def showEvent(self, *args, **kwargs) -> None:
        """
        Detect window show event to perform necessary actions. Modern OSes put minimized DE applications into some kind
        of 'sleep' mode to optimize its own performance and energy consumption. This leads to, for example, QT timers to
        be slow down and so on. And it may lead to some violations in a work flow such as connection breaks and stream
        overflows. So it better to 'freeze' any network and graphical activity during such period and then smoothly
        resume it when needed. The user doesn't stare at the application, anyway :)

        Note: this doesn't include simple moving a window to the background (e.g. overlapping by another window)

        :param args: positional arguments passing directly to the base-class function
        :param kwargs: keyword arguments passing directly to the base-class function
        :return: None
        """

        # 1. resume the listening section of the input thread first
        self.app.conn.resume()

        # 2. start the connection check timer
        if not self.app.isOfflineMode:
            self.app.connCheckTimer.start(self.app.settings['network']['checkInterval'])

        # 3. re-run graphs if they were in run
        if self.graphsWereRun:
            self.playpauseGraphs()

        # finally, call the base class' method
        super(MainWindow, self).showEvent(*args, **kwargs)


    def closeEvent(self, event) -> None:
        """
        Catch the window close event - interpret it the same as an application termination

        :param event: QT event
        :return: None
        """

        self.hide()
        self.app.quit()




class MainApplication(QApplication):
    """
    Customized QApplication - entry point of the whole program
    """


    connLostSignal = pyqtSignal()  # must be part of the class definition and cannot be dynamically added after


    def __init__(self, argv: list):
        """
        MainApplication constructor

        :param argv: the list of command line arguments passed to a Python script
        """

        super(MainApplication, self).__init__(argv)


        # settings [customized] dictionary
        d = Path(__file__).resolve().parent.parent
        self.settings = settings.Settings(defaults=util.resource_path(d/'defaultSettings.json'))
        #self.settings = settings.Settings(defaults=util.resource_path('../defaultSettings.json'))

        if self.settings['appearance']['theme'] == 'dark':
            # TODO: warns itself as a deprecated method though no suitable alternative has been suggested
            self.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())


        # Create a handler function for connection breaks (for example, when a break is occur during the read of some
        # coefficient from the controller). We need to setup it early here to proper connection initialization process
        self.connLostSignal.connect(self.connLostHandler)
        # show this when the connection is broken
        self.connLostStatusBarLabel = QLabel("<font color='red'>Connection was lost. Trying to reconnect...</font>")
        # check connection timer, set it when we will know status of the connection
        self.connCheckTimer = QTimer()

        self.isOfflineMode = False

        self.conn = remotecontroller.RemoteController(
            self.settings['network']['ip'],
            self.settings['network']['port'],
            conn_lost_signal=self.connLostSignal
        )

        # RemoteController' self-check determines the state of the connection. Such app state determined during the
        # startup process will remain during all following activities (i.e. app enters the demo mode) and can be changed
        # only after the restart
        if self.conn.is_offline_mode:
            self.isOfflineMode = True
            print("Offline mode")
            miscgraphics.MessageWindow("No connection to the remote controller. App goes to the Offline (demo) mode. "
                                "All values are random. To try to reconnect please restart the app", status='Warning')

        else:
            # If connection is present (so no demo mode is needed) then create the timer for connection checking. It
            # will start on MainWindow' show
            self.connCheckTimer.timeout.connect(self.connCheckTimerHandler)

        self.conn.save_current_values()


        # We can create the MainWindow only after instantiating the RemoteController because it is used for obtaining
        # values and setting parameters
        self.mainWindow = MainWindow(app=self)
        self.mainWindow.show()


    def quit(self) -> None:
        """
        QApplication quit method

        :return: None
        """

        self.connLostSignal.disconnect(self.connLostHandler)
        self.connCheckTimer.stop()

        self.conn.close()

        super(MainApplication, self).quit()


    def connCheckTimerHandler(self) -> None:
        """
        Procedure to invoke for connection checking

        :return: None
        """

        print("Check connection")

        if self.conn.check_connection() == remotecontroller.result['error']:
            self.connLostHandler()
        else:
            # prevent of multiple calls of these instructions by using this flag
            if self.isOfflineMode:
                self.isOfflineMode = False
                print('Reconnected')
                self.mainWindow.centralWidget.updateDisplayingValues()
                self.mainWindow.statusBar().removeWidget(self.connLostStatusBarLabel)
                self.mainWindow.statusBar().showMessage('Reconnected')


    @pyqtSlot()
    def connLostHandler(self) -> None:
        """
        Slot corresponding to MainApplication.connLostSignal (typically emitted from RemoteController on unsuccessful
        read/write operations). Can also be called directly to handle connection break case

        :return: None
        """

        if not self.isOfflineMode:
            self.isOfflineMode = True
            print("Connection lost")
            try:
                if self.mainWindow.centralWidget.graphs.isRun:
                    self.mainWindow.playpauseGraphs()
                self.mainWindow.statusBar().addWidget(self.connLostStatusBarLabel)
                miscgraphics.MessageWindow("Connection was lost. The app goes to the Offline mode and will be trying "
                                           "to reconnect", status='Warning')

            # This exception is met when the RemoteController.check_connection() test has been passed but following
            # read/write operations cannot be successfully performed due to small corresponding timeouts. Then, connLost
            # signal is emitting. Since the RemoteController initialization has not been complete MainWindow certainly
            # has not been instantiated yet and so we will catch this exception. We can simply ignore such case and just
            # quietly mark the connection as offline without any notifications toward the user but we do
            except AttributeError:
                print("Too small timeout")
                miscgraphics.MessageWindow("It seems like the connection is present but specified read/write timeouts "
                                           "is too small. Consider to fix your network or edit timeouts to match your "
                                           f"timings (see '{ remotecontroller.__name__}.py' module).", status='Warning')




if __name__ == '__main__':
    """
    Main entry point
    """

    multiprocessing.freeze_support()  # Windows support

    QCoreApplication.setOrganizationName("Andrey Chufyrev")
    QCoreApplication.setApplicationName("PID controller GUI")

    application = MainApplication(sys.argv)

    sys.exit(application.exec_())
