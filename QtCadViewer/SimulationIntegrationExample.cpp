#include "ThreadedSimulationManager.h"
#include <QMainWindow>
#include <QToolBar>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QTimer>
#include <QMessageBox>
#include <QMetaObject>

// Example GUI class that integrates with ThreadedSimulationManager
class SimulationGUI : public QMainWindow {
    Q_OBJECT

public:
    SimulationGUI(QWidget* parent = nullptr) : QMainWindow(parent) {
        setupUI();
        setupSimulation();
        setupTimer();
    }

    ~SimulationGUI() {
        if (m_simulationManager) {
            m_simulationManager->stopSimulation();
        }
    }

private slots:
    void onStartClicked() {
        if (m_simulationManager) {
            m_simulationManager->startSimulation();
            updateButtonStates();
        }
    }

    void onPauseClicked() {
        if (m_simulationManager) {
            if (m_simulationManager->isPaused()) {
                m_simulationManager->resumeSimulation();
                m_pauseButton->setText("Pause");
            } else {
                m_simulationManager->pauseSimulation();
                m_pauseButton->setText("Resume");
            }
        }
    }

    void onResetClicked() {
        if (m_simulationManager) {
            m_simulationManager->resetSimulation();
        }
    }

    void onStepClicked() {
        if (m_simulationManager) {
            m_simulationManager->stepSimulation();
        }
    }

    void onAddObjectClicked() {
        if (m_simulationManager) {
            // Example: Add a simple physics object
            PhysicsObject obj;
            obj.id = "object_" + std::to_string(m_objectCounter++);
            obj.position[0] = 0.0f;
            obj.position[1] = 10.0f;  // Start above ground
            obj.position[2] = 0.0f;
            obj.velocity[0] = 0.0f;
            obj.velocity[1] = 0.0f;
            obj.velocity[2] = 0.0f;
            obj.mass = 1.0f;
            obj.isKinematic = false;

            m_simulationManager->addObject(obj);
        }
    }

    void onSimulationUpdate(const SimulationState& state) {
        // This callback runs in the simulation thread
        // We need to schedule the GUI update on the main thread
        QMetaObject::invokeMethod(this, "updateGUIFromSimulation", 
                                 Qt::QueuedConnection,
                                 Q_ARG(SimulationState, state));
    }

    void updateGUIFromSimulation(const SimulationState& state) {
        // This method runs on the main thread
        m_timeLabel->setText(QString("Time: %1 s").arg(state.time, 0, 'f', 2));
        m_stepLabel->setText(QString("Steps: %1").arg(state.stepCount));
        m_objectCountLabel->setText(QString("Objects: %1").arg(state.objects.size()));

        // Update your 3D view or other GUI elements here
        update3DView(state);
    }

    void updateTimer() {
        // Optional: Update GUI periodically even without simulation updates
        if (m_simulationManager) {
            auto state = m_simulationManager->getCurrentState();
            updateGUIFromSimulation(state);
        }
    }

private:
    void setupUI() {
        // Create central widget
        QWidget* centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);

        // Create toolbar
        QToolBar* toolbar = new QToolBar("Simulation Controls", this);
        addToolBar(toolbar);

        // Create control buttons
        m_startButton = new QPushButton("Start", toolbar);
        m_pauseButton = new QPushButton("Pause", toolbar);
        m_resetButton = new QPushButton("Reset", toolbar);
        m_stepButton = new QPushButton("Step", toolbar);
        m_addObjectButton = new QPushButton("Add Object", toolbar);

        // Add buttons to toolbar
        toolbar->addWidget(m_startButton);
        toolbar->addWidget(m_pauseButton);
        toolbar->addWidget(m_resetButton);
        toolbar->addWidget(m_stepButton);
        toolbar->addSeparator();
        toolbar->addWidget(m_addObjectButton);

        // Create status labels
        QWidget* statusWidget = new QWidget(centralWidget);
        QHBoxLayout* statusLayout = new QHBoxLayout(statusWidget);
        
        m_timeLabel = new QLabel("Time: 0.00 s", statusWidget);
        m_stepLabel = new QLabel("Steps: 0", statusWidget);
        m_objectCountLabel = new QLabel("Objects: 0", statusWidget);

        statusLayout->addWidget(m_timeLabel);
        statusLayout->addWidget(m_stepLabel);
        statusLayout->addWidget(m_objectCountLabel);
        statusLayout->addStretch();

        // Create main layout
        QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);
        mainLayout->addWidget(statusWidget);
        
        // Add your 3D view widget here
        // mainLayout->addWidget(m_3dViewWidget);

        // Connect button signals
        connect(m_startButton, &QPushButton::clicked, this, &SimulationGUI::onStartClicked);
        connect(m_pauseButton, &QPushButton::clicked, this, &SimulationGUI::onPauseClicked);
        connect(m_resetButton, &QPushButton::clicked, this, &SimulationGUI::onResetClicked);
        connect(m_stepButton, &QPushButton::clicked, this, &SimulationGUI::onStepClicked);
        connect(m_addObjectButton, &QPushButton::clicked, this, &SimulationGUI::onAddObjectClicked);

        // Set initial button states
        updateButtonStates();

        setWindowTitle("Physics Simulation");
        resize(800, 600);
    }

    void setupSimulation() {
        m_simulationManager = std::make_unique<ThreadedSimulationManager>();
        
        // Set up callback for simulation updates
        m_simulationManager->setUpdateCallback(
            [this](const SimulationState& state) {
                onSimulationUpdate(state);
            }
        );

        // Set initial simulation parameters
        m_simulationManager->updateSimulationParameters(-9.81f, 1.0f/60.0f, 0.1f);
    }

    void setupTimer() {
        // Optional: Set up a timer for periodic GUI updates
        m_updateTimer = new QTimer(this);
        connect(m_updateTimer, &QTimer::timeout, this, &SimulationGUI::updateTimer);
        m_updateTimer->start(16); // ~60 FPS
    }

    void updateButtonStates() {
        if (!m_simulationManager) {
            m_startButton->setEnabled(false);
            m_pauseButton->setEnabled(false);
            m_resetButton->setEnabled(false);
            m_stepButton->setEnabled(false);
            m_addObjectButton->setEnabled(false);
            return;
        }

        bool isRunning = m_simulationManager->isRunning();
        bool isPaused = m_simulationManager->isPaused();

        m_startButton->setEnabled(!isRunning);
        m_pauseButton->setEnabled(isRunning);
        m_resetButton->setEnabled(true);
        m_stepButton->setEnabled(isRunning && isPaused);
        m_addObjectButton->setEnabled(true);

        if (isRunning && isPaused) {
            m_pauseButton->setText("Resume");
        } else {
            m_pauseButton->setText("Pause");
        }
    }

    void update3DView(const SimulationState& state) {
        // Update your 3D view with the simulation state
        // This is where you would update your OpenGL widget or other 3D rendering
        
        // Example: Update object positions in your 3D scene
        for (const auto& obj : state.objects) {
            // Find the corresponding 3D object and update its position
            // m_3dViewWidget->updateObjectPosition(obj.id, obj.position);
            
            // Example output (replace with actual 3D updates)
            if (obj.id == "object_0") {  // Just for demonstration
                // Update the first object's position in your 3D view
            }
        }
    }

private:
    std::unique_ptr<ThreadedSimulationManager> m_simulationManager;
    
    // GUI elements
    QPushButton* m_startButton;
    QPushButton* m_pauseButton;
    QPushButton* m_resetButton;
    QPushButton* m_stepButton;
    QPushButton* m_addObjectButton;
    
    QLabel* m_timeLabel;
    QLabel* m_stepLabel;
    QLabel* m_objectCountLabel;
    
    QTimer* m_updateTimer;
    
    int m_objectCounter = 0;
};

// Example usage in main.cpp or main window
/*
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    SimulationGUI window;
    window.show();
    
    return app.exec();
}
*/ 