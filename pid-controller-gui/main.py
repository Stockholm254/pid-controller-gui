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
import time

from PyQt5.QtCore import Qt, QCoreApplication, QTimer, pyqtSlot, pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QGridLayout, QHBoxLayout, QLabel, QAction
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



DERIVATIVE_LABEL_UPDATE_INTERVAL_MS = 1000  # refresh period of the dT/dt label (no controller traffic involved)
DERIVATIVE_WINDOW_LIMITS_S = (2.0, 120.0)  # same clamping as the controller applies to 'd_window'
TEMPERATURE_MAX_AGE_S = 5.0  # newest streamed temperature older than this is not considered current



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
        self.setpointBox = self.contValGroupBoxes[0]
        self.dWindowBox = self.contValGroupBoxes[4]

        for groupBox, yPosition in zip(self.contValGroupBoxes, [0,3,6,9,12]):
            grid.addWidget(groupBox, yPosition, 0, 3, 2)


        self.graphs = graphs.CustomGraphicsLayoutWidget(
            names=(app.settings['pid']['processVariable']['name'],
                   app.settings['pid']['controllerOutput']['name']),
            numPoints=app.settings['graphs']['numberOfPoints'],
            interval=app.settings['graphs']['updateInterval'],
            ranges=((app.settings['pid']['processVariable']['limits']['min'],
                     app.settings['pid']['processVariable']['limits']['max']),
                    (app.settings['pid']['controllerOutput']['limits']['min'],
                     app.settings['pid']['controllerOutput']['limits']['max'])),
            units=(app.settings['pid']['processVariable']['unit'],
                   app.settings['pid']['controllerOutput']['unit']),
            controlPipe=None if app.isOfflineMode else app.conn.input_thread_control_pipe_main,
            streamPipeRX=None if app.isOfflineMode else app.conn.stream.pipe_rx,
            theme=app.settings['appearance']['theme'],
        )

        for averageLabel, name, yPosition in zip(self.graphs.averageLabels, self.graphs.names, [15,16]):
            hBox = QHBoxLayout()
            hBox.addWidget(QLabel(name))
            hBox.addWidget(averageLabel, alignment=Qt.AlignLeft)
            grid.addLayout(hBox, yPosition, 0, 1, 2)

        # smooth derivative of the temperature over the same window the controller uses for its D term
        self.derivativeLabel = QLabel()
        hBox = QHBoxLayout()
        hBox.addWidget(QLabel("dT/dt"))
        hBox.addWidget(self.derivativeLabel, alignment=Qt.AlignLeft)
        grid.addLayout(hBox, 17, 0, 1, 2)

        grid.addWidget(self.graphs, 0, 2, 18, 6)

        self.derivativeTimer = QTimer()
        self.derivativeTimer.timeout.connect(self.updateDerivativeLabel)
        self.derivativeTimer.start(DERIVATIVE_LABEL_UPDATE_INTERVAL_MS)
        self.updateDerivativeLabel()


    def derivativeWindow(self) -> float:
        """
        Window (in seconds) of the controller's smoothed derivative, as last read from or written to the controller

        :return: window length in seconds
        """

        window = self.dWindowBox.value
        if window is None:
            window = remotecontroller.snapshot_template['d_window']
        return min(max(window, DERIVATIVE_WINDOW_LIMITS_S[0]), DERIVATIVE_WINDOW_LIMITS_S[1])


    def _fitRecentTemperatures(self):
        """
        Straight-line fit of the streamed temperatures from the last derivative window

        :return: util.fit_line() result or None if there is no current data to fit
        """

        samples = self.graphs.tempSamples
        now = time.monotonic()
        if not samples or now - samples[-1][0] > TEMPERATURE_MAX_AGE_S:
            return None

        window = self.derivativeWindow()
        return util.fit_line((t, value) for t, value in samples if now - t <= window)


    def smoothedTemperature(self) -> float:
        """
        Current temperature smoothed over the derivative window

        :return: temperature or None if no current data is streamed
        """

        fit = self._fitRecentTemperatures()
        if fit is not None:
            return fit[1]

        samples = self.graphs.tempSamples
        if samples and time.monotonic() - samples[-1][0] <= TEMPERATURE_MAX_AGE_S:
            return samples[-1][1]  # too few points in the window to fit a line - use the newest one
        return None


    def updateDerivativeLabel(self) -> None:
        """
        Show the smooth derivative of the temperature (callback for derivativeTimer.timeout slot)

        :return: None
        """

        fit = self._fitRecentTemperatures()
        if fit is None:
            self.derivativeLabel.setText("—")
        else:
            self.derivativeLabel.setText(f"{fit[0] * 60:+.3f} °C/min ({self.derivativeWindow():g} s fit)")


    def updateDisplayingValues(self) -> None:
        """
        Retrieve all controller parameters and update corresponding GUI elements. Useful to apply after connection's
        breaks. This does not affect values saved during the app launch (RemoteController.save_current_values())

        :return: None
        """

        for groupBox in self.contValGroupBoxes:
            groupBox.refreshVal()

        self.app.mainWindow.errorsSettingsWindow.updateDisplayingValues('err_P_limits', 'err_I_limits')




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

        dampAction = QAction(QIcon(util.resource_path('../img/play_pause.png')), 'Derivative damp', self)
        dampAction.setShortcut('D')
        dampAction.setStatusTip("[D] Lock at the current temperature and damp its changes (dT/dt -> 0)")
        dampAction.triggered.connect(self.dampPID)

        playpauseAction = QAction(QIcon(util.resource_path('../img/play_pause.png')), 'Play/Pause', self)
        playpauseAction.setShortcut('P')
        playpauseAction.setStatusTip("[P] Play/pause graphs")
        playpauseAction.triggered.connect(self.playpauseGraphs)

        graphsToolbar = self.addToolBar('graphs')  # internal name
        graphsToolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        graphsToolbar.addAction(lockAction)
        graphsToolbar.addAction(dampAction)
        graphsToolbar.addAction(playpauseAction)
        self.playpauseButton = graphsToolbar.widgetForAction(playpauseAction)
        self.playpauseButton.setCheckable(True)
        self.playpauseButton.setChecked(True)
        self.graphsWereRun = False

        self.lockButton = graphsToolbar.widgetForAction(lockAction)
        self.lockButton.setCheckable(True)
        self.lockButton.setChecked(False)

        self.dampButton = graphsToolbar.widgetForAction(dampAction)
        self.dampButton.setCheckable(True)
        self.dampButton.setChecked(False)

        mainMenu = self.menuBar().addMenu('&Menu')
        mainMenu.addAction(aboutAction)
        mainMenu.addAction(settingsAction)
        mainMenu.addAction(exitAction)


        if self.app.isOfflineMode:
            self.statusBar().addWidget(QLabel("<font color='red'>Offline mode</font>"))

        self.centralWidget = CentralWidget(app=app)
        self.setCentralWidget(self.centralWidget)

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
                self.dampButton.setChecked(False)
        else:
            print("start lock")
            if self.app.conn.start_lock() == remotecontroller.result['ok']:
                self.lockButton.setChecked(True)

    def dampPID(self) -> None:
        """
        Toggles the derivative damping. Turning it on locks the PID at the current (smoothed) temperature: the
        controller's smoothed D term then damps temperature changes and, with kI > 0, the integral cancels the remaining
        drift (dT/dt -> 0). With kP = kI = 0 it is a pure D damper. Turning it off unlocks the PID

        :return: None
        """

        if self.dampButton.isChecked():
            print("stop derivative damp")
            if self.app.conn.stop_lock() == remotecontroller.result['ok']:
                self.dampButton.setChecked(False)
                self.lockButton.setChecked(False)
            return

        if self.app.isOfflineMode:
            miscgraphics.MessageWindow("Derivative damp needs a connection to the controller", status='Warning')
            return

        temperature = self.centralWidget.smoothedTemperature()
        if temperature is None:
            miscgraphics.MessageWindow("No live temperature. Start the graphs [P] and try again", status='Warning')
            return

        print(f"start derivative damp at {temperature}")
        # an exception escaping a Qt slot would abort the whole app
        try:
            if self.app.conn.write('setpoint', temperature) != remotecontroller.result['ok']:
                return  # connection is lost, the user has already been notified
            self.centralWidget.setpointBox.showVal(temperature)

            if not self.lockButton.isChecked():
                if self.app.conn.start_lock() != remotecontroller.result['ok']:
                    return
                self.lockButton.setChecked(True)
        except (remotecontroller.ResponseException, remotecontroller.ResponseVarCmdMismatchException,
                remotecontroller.ResponseOperationMismatchException) as e:
            miscgraphics.MessageWindow(f"Derivative damp failed: {e}", status='Error')
            return

        self.dampButton.setChecked(True)
        self.statusBar().showMessage(f"Derivative damp: holding dT/dt at 0 around {temperature:.3f}")


    def restoreContValues(self) -> None:
        """
        Write PID parameters stored at program start to the controller

        :return: None
        """

        self.dampButton.setChecked(False)  # the setpoint captured by the derivative damp is overwritten
        date = self.app.conn.restore_values(self.app.conn.snapshots[0])  # currently save and use only one snapshot
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
