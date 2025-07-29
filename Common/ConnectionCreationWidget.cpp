#include "ConnectionCreationWidget.h"
#include "HelperFunctions.h"
#include "CadOpenGLWidget.h"
#include "SimulationManager.h"
#include "DragChainConstraintSolver.h"
#include <QApplication>
#include <QScreen>
#include <QVector3D>
#include <cmath>

ConnectionCreationWidget::ConnectionCreationWidget(CustomModelTreeModel* model, QWidget* parent)
    : QWidget(parent)
    , m_model(model)
    , m_point1()
    , m_point2()
{
    setWindowTitle("Create Connection");
    resize(600, 500);
    
    setupUI();
    populateConnectionPoints();
}

void ConnectionCreationWidget::setupUI()
{
    auto mainLayout = new QVBoxLayout(this);
    
    // Connection points selection
    auto pointsGroup = new QGroupBox("Connection Points");
    auto pointsLayout = new QHBoxLayout(pointsGroup);
    
    // Point 1 selection
    auto point1Layout = new QVBoxLayout();
    point1Layout->addWidget(new QLabel("Point 1:"));
    m_point1List = new QListWidget();
    m_point1List->setMaximumHeight(150);
    point1Layout->addWidget(m_point1List);
    pointsLayout->addLayout(point1Layout);
    
    // Point 2 selection
    auto point2Layout = new QVBoxLayout();
    point2Layout->addWidget(new QLabel("Point 2:"));
    m_point2List = new QListWidget();
    m_point2List->setMaximumHeight(150);
    point2Layout->addWidget(m_point2List);
    pointsLayout->addLayout(point2Layout);
    
    mainLayout->addWidget(pointsGroup);
    
    // Connection properties
    auto propertiesGroup = new QGroupBox("Connection Properties");
    auto propertiesLayout = new QFormLayout(propertiesGroup);
    
    m_connectionNameEdit = new QLineEdit();
    m_connectionNameEdit->setText("New Connection");
    propertiesLayout->addRow("Name:", m_connectionNameEdit);
    
    m_connectionTypeCombo = new QComboBox();
    m_connectionTypeCombo->addItem("Cable", static_cast<int>(ConnectionNodeData::ConnectionType::Cable));
    m_connectionTypeCombo->addItem("Drag Chain", static_cast<int>(ConnectionNodeData::ConnectionType::DragChain));
    m_connectionTypeCombo->addItem("Conveyor", static_cast<int>(ConnectionNodeData::ConnectionType::Conveyor));
    m_connectionTypeCombo->addItem("Hose", static_cast<int>(ConnectionNodeData::ConnectionType::Hose));
    m_connectionTypeCombo->addItem("Wire", static_cast<int>(ConnectionNodeData::ConnectionType::Wire));
    m_connectionTypeCombo->setCurrentIndex(1); // Set Drag Chain as default
    propertiesLayout->addRow("Type:", m_connectionTypeCombo);
    
    m_isFlexibleCheck = new QCheckBox("Flexible Connection");
    m_isFlexibleCheck->setChecked(true);
    propertiesLayout->addRow("", m_isFlexibleCheck);
    
    m_maxBendRadiusSpin = new QDoubleSpinBox();
    m_maxBendRadiusSpin->setRange(0.0, 1000.0);
    m_maxBendRadiusSpin->setValue(18.0);
    m_maxBendRadiusSpin->setSuffix(" mm");
    propertiesLayout->addRow("Max Bend Radius:", m_maxBendRadiusSpin);
    
    m_pitchLengthSpin = new QDoubleSpinBox();
    m_pitchLengthSpin->setRange(10.0, 1000.0);
    m_pitchLengthSpin->setValue(18.0);
    m_pitchLengthSpin->setSuffix(" mm");
    m_pitchLengthSpin->setToolTip("Length of each drag chain segment");
    propertiesLayout->addRow("Pitch Length:", m_pitchLengthSpin);
    
    m_segmentCountSpin = new QSpinBox();
    m_segmentCountSpin->setRange(1, 50);
    m_segmentCountSpin->setValue(3);
    m_segmentCountSpin->setSuffix(" segments");
    m_segmentCountSpin->setToolTip("Total number of segments for the drag chain");
    propertiesLayout->addRow("Segment Count:", m_segmentCountSpin);
    
    // Solver parameter controls
    m_maxIterationsSpin = new QSpinBox();
    m_maxIterationsSpin->setRange(1, 1000000);
    m_maxIterationsSpin->setValue(100);
    m_maxIterationsSpin->setSuffix(" iterations");
    m_maxIterationsSpin->setToolTip("Maximum number of solver iterations");
    propertiesLayout->addRow("Max Iterations:", m_maxIterationsSpin);
    
    m_convergenceToleranceSpin = new QDoubleSpinBox();
    m_convergenceToleranceSpin->setRange(0.001, 10.0);
    m_convergenceToleranceSpin->setValue(0.01);
    m_convergenceToleranceSpin->setSuffix(" mm");
    m_convergenceToleranceSpin->setDecimals(3);
    m_convergenceToleranceSpin->setToolTip("Convergence tolerance for solver");
    propertiesLayout->addRow("Convergence Tolerance:", m_convergenceToleranceSpin);
    
    m_simulationTimeStepSpin = new QDoubleSpinBox();
    m_simulationTimeStepSpin->setRange(0.001, 0.1);
    m_simulationTimeStepSpin->setValue(1.0/60.0);
    m_simulationTimeStepSpin->setSuffix(" s");
    m_simulationTimeStepSpin->setDecimals(4);
    m_simulationTimeStepSpin->setToolTip("Physics simulation time step");
    propertiesLayout->addRow("Simulation Time Step:", m_simulationTimeStepSpin);
    
    mainLayout->addWidget(propertiesGroup);
    
    // Attachment configuration for drag chains
    auto attachmentGroup = new QGroupBox("Attachment Configuration");
    auto attachmentLayout = new QHBoxLayout(attachmentGroup);
    
    // Start attachment configuration
    m_startAttachmentGroup = new QGroupBox("Start Attachment");
    auto startAttachmentLayout = new QFormLayout(m_startAttachmentGroup);
    
    m_lockStartAttachmentCheck = new QCheckBox("Lock Start Attachment");
    m_lockStartAttachmentCheck->setToolTip("Lock the plane and direction for the start of the drag chain");
    startAttachmentLayout->addRow("", m_lockStartAttachmentCheck);
    
    m_startPlaneCombo = new QComboBox();
    m_startPlaneCombo->addItem("XY Plane (Z+)", 0);
    m_startPlaneCombo->addItem("XY Plane (Z-)", 1);
    m_startPlaneCombo->addItem("XZ Plane (Y+)", 2);
    m_startPlaneCombo->addItem("XZ Plane (Y-)", 3);
    m_startPlaneCombo->addItem("YZ Plane (X+)", 4);
    m_startPlaneCombo->addItem("YZ Plane (X-)", 5);
    m_startPlaneCombo->setToolTip("Plane normal for start attachment");
    startAttachmentLayout->addRow("Plane Normal:", m_startPlaneCombo);
    
    m_startDirectionCombo = new QComboBox();
    m_startDirectionCombo->addItem("X+ Direction", 0);
    m_startDirectionCombo->addItem("X- Direction", 1);
    m_startDirectionCombo->addItem("Y+ Direction", 2);
    m_startDirectionCombo->addItem("Y- Direction", 3);
    m_startDirectionCombo->addItem("Z+ Direction", 4);
    m_startDirectionCombo->addItem("Z- Direction", 5);
    m_startDirectionCombo->setToolTip("Direction vector for start attachment");
    startAttachmentLayout->addRow("Direction:", m_startDirectionCombo);
    
    attachmentLayout->addWidget(m_startAttachmentGroup);
    
    // End attachment configuration
    m_endAttachmentGroup = new QGroupBox("End Attachment");
    auto endAttachmentLayout = new QFormLayout(m_endAttachmentGroup);
    
    m_lockEndAttachmentCheck = new QCheckBox("Lock End Attachment");
    m_lockEndAttachmentCheck->setToolTip("Lock the plane and direction for the end of the drag chain");
    endAttachmentLayout->addRow("", m_lockEndAttachmentCheck);
    
    m_endPlaneCombo = new QComboBox();
    m_endPlaneCombo->addItem("XY Plane (Z+)", 0);
    m_endPlaneCombo->addItem("XY Plane (Z-)", 1);
    m_endPlaneCombo->addItem("XZ Plane (Y+)", 2);
    m_endPlaneCombo->addItem("XZ Plane (Y-)", 3);
    m_endPlaneCombo->addItem("YZ Plane (X+)", 4);
    m_endPlaneCombo->addItem("YZ Plane (X-)", 5);
    m_endPlaneCombo->setToolTip("Plane normal for end attachment");
    endAttachmentLayout->addRow("Plane Normal:", m_endPlaneCombo);
    
    m_endDirectionCombo = new QComboBox();
    m_endDirectionCombo->addItem("X+ Direction", 0);
    m_endDirectionCombo->addItem("X- Direction", 1);
    m_endDirectionCombo->addItem("Y+ Direction", 2);
    m_endDirectionCombo->addItem("Y- Direction", 3);
    m_endDirectionCombo->addItem("Z+ Direction", 4);
    m_endDirectionCombo->addItem("Z- Direction", 5);
    m_endDirectionCombo->setToolTip("Direction vector for end attachment");
    endAttachmentLayout->addRow("Direction:", m_endDirectionCombo);
    
    attachmentLayout->addWidget(m_endAttachmentGroup);
    
    mainLayout->addWidget(attachmentGroup);
    
    // Connection info
    auto infoGroup = new QGroupBox("Connection Information");
    auto infoLayout = new QFormLayout(infoGroup);
    
    m_distanceLabel = new QLabel("Distance: --");
    infoLayout->addRow("Distance:", m_distanceLabel);
    
    m_calculatedLengthLabel = new QLabel("Calculated Length: --");
    infoLayout->addRow("Calculated Length:", m_calculatedLengthLabel);
    
    m_effectiveLengthLabel = new QLabel("Effective Length: --");
    infoLayout->addRow("Effective Length:", m_effectiveLengthLabel);
    
    m_compatibilityLabel = new QLabel("Compatibility: --");
    infoLayout->addRow("Compatibility:", m_compatibilityLabel);
    
    m_placementLabel = new QLabel("Placement: --");
    infoLayout->addRow("Placement:", m_placementLabel);
    
    m_segmentLengthsLabel = new QLabel("Segment Lengths: --");
    infoLayout->addRow("Segment Lengths:", m_segmentLengthsLabel);
    
    m_solverIterationsLabel = new QLabel("Solver Iterations: --");
    infoLayout->addRow("Solver Iterations:", m_solverIterationsLabel);
    
    mainLayout->addWidget(infoGroup);
    
    // Buttons
    auto buttonLayout = new QHBoxLayout();
    
    m_createButton = new QPushButton("Create Connection");
    m_createButton->setEnabled(false);
    buttonLayout->addWidget(m_createButton);
    
    m_updateVisualizationButton = new QPushButton("Update Visualization");
    m_updateVisualizationButton->setToolTip("Manually update the connection path visualization");
    buttonLayout->addWidget(m_updateVisualizationButton);
    
    m_cancelButton = new QPushButton("Cancel");
    buttonLayout->addWidget(m_cancelButton);
    
    mainLayout->addLayout(buttonLayout);
    
    // Connect signals
    connect(m_point1List, &QListWidget::currentRowChanged, this, &ConnectionCreationWidget::onPoint1SelectionChanged);
    connect(m_point2List, &QListWidget::currentRowChanged, this, &ConnectionCreationWidget::onPoint2SelectionChanged);
    connect(m_connectionTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ConnectionCreationWidget::onConnectionTypeChanged);
    connect(m_isFlexibleCheck, &QCheckBox::toggled, this, &ConnectionCreationWidget::onConnectionTypeChanged);
    connect(m_maxBendRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ConnectionCreationWidget::onConnectionTypeChanged);
    connect(m_pitchLengthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ConnectionCreationWidget::onConnectionTypeChanged);
    connect(m_segmentCountSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &ConnectionCreationWidget::onConnectionTypeChanged);
    
    // Connect attachment configuration signals
    connect(m_lockStartAttachmentCheck, &QCheckBox::toggled, this, &ConnectionCreationWidget::onAttachmentConfigChanged);
    connect(m_lockEndAttachmentCheck, &QCheckBox::toggled, this, &ConnectionCreationWidget::onAttachmentConfigChanged);
    connect(m_startPlaneCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ConnectionCreationWidget::onAttachmentConfigChanged);
    connect(m_endPlaneCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ConnectionCreationWidget::onAttachmentConfigChanged);
    connect(m_startDirectionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ConnectionCreationWidget::onAttachmentConfigChanged);
    connect(m_endDirectionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ConnectionCreationWidget::onAttachmentConfigChanged);
    
    connect(m_createButton, &QPushButton::clicked, this, &ConnectionCreationWidget::onCreateConnection);
    connect(m_updateVisualizationButton, &QPushButton::clicked, this, &ConnectionCreationWidget::onUpdateVisualization);
    connect(m_cancelButton, &QPushButton::clicked, this, &ConnectionCreationWidget::onCancel);
}

void ConnectionCreationWidget::populateConnectionPoints()
{
    m_availablePoints.clear();
    m_originalConnectionPoints.clear();
    m_point1List->clear();
    m_point2List->clear();
    
    if (!m_model) {
        qDebug() << "[ConnectionWidget] No model available for populating connection points";
        return;
    }
    
    CadNode* rootNode = const_cast<CadNode*>(m_model->getRootNodePointer());
    if (!rootNode) {
        qDebug() << "[ConnectionWidget] No root node available for populating connection points";
        return;
    }
    
    qDebug() << "[ConnectionWidget] Finding connection points in tree...";
    
    // Find all connection points in the tree
    auto connectionPoints = findConnectionPointsInTree(rootNode);
    
    qDebug() << "[ConnectionWidget] Found" << connectionPoints.size() << "connection points";
    
    for (auto* point : connectionPoints) {
        // Create a deep copy for the widget to own
        auto pointCopy = deepCopyNodeNonExcluded(point);
        m_availablePoints.push_back(pointCopy);
        
        // Store reference to original node for finding locations in tree
        m_originalConnectionPoints.push_back(point);
        
        // Add to both lists
        QString displayName = QString::fromStdString(point->name);
        m_point1List->addItem(displayName);
        m_point2List->addItem(displayName);
        qDebug() << "[ConnectionWidget] Added connection point:" << displayName;
    }
    
    qDebug() << "[ConnectionWidget] Populated" << m_availablePoints.size() << "connection points";
}

void ConnectionCreationWidget::onPoint1SelectionChanged()
{
    int index = m_point1List->currentRow();
    qDebug() << "[ConnectionWidget] Point1 selection changed, index:" << index;
    if (index >= 0 && index < static_cast<int>(m_availablePoints.size())) {
        m_point1 = m_availablePoints[index];
        qDebug() << "[ConnectionWidget] Point1 set to:" << QString::fromStdString(m_point1->name);
    } else {
        m_point1.reset();
        qDebug() << "[ConnectionWidget] Point1 cleared";
    }
    
    qDebug() << "[ConnectionWidget] Clearing solver statistics due to point1 change";
    clearSolverStatistics(); // Clear statistics when points change
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onPoint2SelectionChanged()
{
    int index = m_point2List->currentRow();
    qDebug() << "[ConnectionWidget] Point2 selection changed, index:" << index;
    if (index >= 0 && index < static_cast<int>(m_availablePoints.size())) {
        m_point2 = m_availablePoints[index];
        qDebug() << "[ConnectionWidget] Point2 set to:" << QString::fromStdString(m_point2->name);
    } else {
        m_point2.reset();
        qDebug() << "[ConnectionWidget] Point2 cleared";
    }
    
    qDebug() << "[ConnectionWidget] Clearing solver statistics due to point2 change";
    clearSolverStatistics(); // Clear statistics when points change
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onConnectionTypeChanged(int index)
{
    Q_UNUSED(index)
    
    // Only clear solver statistics if we're changing to a different connection type
    // or if we don't have valid statistics yet
    if (!hasValidSolverStatistics()) {
        clearSolverStatistics();
    } else {
        qDebug() << "[ConnectionWidget] Preserving existing solver statistics during connection type change";
    }
    
    updateConnectionPathVisualization();
    updateSolverStatisticsDisplay();
}

void ConnectionCreationWidget::onAttachmentConfigChanged()
{
    // Only clear solver statistics if we don't have valid statistics yet
    if (!hasValidSolverStatistics()) {
        clearSolverStatistics();
    } else {
        qDebug() << "[ConnectionWidget] Preserving existing solver statistics during attachment config change";
    }
    
    updateConnectionPathVisualization();
    updateSolverStatisticsDisplay();
}

void ConnectionCreationWidget::onUpdateVisualization()
{
    qDebug() << "[ConnectionWidget] Manual visualization update requested";
    updateConnectionPathVisualization();
}

// Helper methods for converting between combo box indices and vectors
QVector3D ConnectionCreationWidget::getPlaneNormalFromIndex(int index) const
{
    switch (index) {
        case 0: return QVector3D(0, 0, 1);   // XY Plane (Z+)
        case 1: return QVector3D(0, 0, -1);  // XY Plane (Z-)
        case 2: return QVector3D(0, 1, 0);   // XZ Plane (Y+)
        case 3: return QVector3D(0, -1, 0);  // XZ Plane (Y-)
        case 4: return QVector3D(1, 0, 0);   // YZ Plane (X+)
        case 5: return QVector3D(-1, 0, 0);  // YZ Plane (X-)
        default: return QVector3D(0, 0, 1);
    }
}

QVector3D ConnectionCreationWidget::getDirectionFromIndex(int index) const
{
    switch (index) {
        case 0: return QVector3D(1, 0, 0);   // X+ Direction
        case 1: return QVector3D(-1, 0, 0);  // X- Direction
        case 2: return QVector3D(0, 1, 0);   // Y+ Direction
        case 3: return QVector3D(0, -1, 0);  // Y- Direction
        case 4: return QVector3D(0, 0, 1);   // Z+ Direction
        case 5: return QVector3D(0, 0, -1);  // Z- Direction
        default: return QVector3D(1, 0, 0);
    }
}

std::vector<CadOpenGLWidget::ConnectionPathSegment> ConnectionCreationWidget::createDragChainPath(
    const QVector3D& point1, const QVector3D& point2, const QVector3D& direction, 
    double distance, double bendRadius, double pitchLength, int segmentCount,
    bool startLocked, bool endLocked)
{
    qDebug() << "[ConnectionWidget] Creating constraint-based drag chain path";
    qDebug() << "[ConnectionWidget] Parameters - distance:" << distance << "pitchLength:" << pitchLength << "segmentCount:" << segmentCount;
    qDebug() << "[ConnectionWidget] Attachments - startLocked:" << startLocked << "endLocked:" << endLocked;
    
    // Create constraint solver instance
    DragChainConstraintSolver solver;
    
    // Set solver parameters from GUI
    solver.setMaxIterations(m_maxIterationsSpin->value());
    solver.setConvergenceTolerance(m_convergenceToleranceSpin->value());
    solver.setBendRadiusTolerance(0.1);
    solver.setSimulationTimeStep(m_simulationTimeStepSpin->value());
        
    // Get attachment configurations
    QVector3D startDirection = direction;
    QVector3D endDirection = direction;
    QVector3D startPlaneNormal(0, 0, 1);
    QVector3D endPlaneNormal(0, 0, 1);
    
    if (startLocked) {
        startDirection = getDirectionFromIndex(m_startDirectionCombo->currentIndex());
        startPlaneNormal = getPlaneNormalFromIndex(m_startPlaneCombo->currentIndex());
    }
        
        if (endLocked) {
        endDirection = getDirectionFromIndex(m_endDirectionCombo->currentIndex());
        endPlaneNormal = getPlaneNormalFromIndex(m_endPlaneCombo->currentIndex());
        }
        
    // Set up callback for real-time statistics updates
    solver.setStatisticsCallback([this](const DragChainConstraintSolver::SolverStatistics& stats) {
        onSolverStatisticsUpdated(stats);
        // Force GUI update to ensure real-time display
        QApplication::processEvents();
        
        // Show progress indicator during solving
        if (stats.iterationsUsed > 0) {
            m_segmentLengthsLabel->setText(QString("Segment Lengths: Min=%1, Max=%2, Avg=%3 mm (Solving...)")
                .arg(stats.minSegmentLength, 0, 'f', 1)
                .arg(stats.maxSegmentLength, 0, 'f', 1)
                .arg(stats.averageSegmentLength, 0, 'f', 1));
            m_solverIterationsLabel->setText(QString("Solver Iterations: %1/%2 (Solving...)")
                .arg(stats.iterationsUsed)
                .arg(m_maxIterationsSpin->value()));
        }
    });
    
    // Solve the drag chain path using OpenCascade constraint solver
    std::vector<DragChainSegment> solvedSegments = solver.solveDragChainPath(
        point1, point2,
        startDirection, endDirection,
        pitchLength, bendRadius, segmentCount,
        startLocked, endLocked,
        startPlaneNormal, endPlaneNormal
    );
                
    // Store solver statistics for later use
    const auto& stats = solver.getLastSolverStatistics();
    qDebug() << "[ConnectionWidget] createDragChainPath: Retrieved solver statistics:";
    qDebug() << "[ConnectionWidget]   Raw stats - Distance to target:" << stats.finalDistanceToTarget << "mm";
    qDebug() << "[ConnectionWidget]   Raw stats - Converged:" << stats.converged;
    qDebug() << "[ConnectionWidget]   Raw stats - Iterations:" << stats.iterationsUsed;
    qDebug() << "[ConnectionWidget]   Raw stats - Simulation time:" << stats.simulationTime << "s";
    qDebug() << "[ConnectionWidget]   Raw stats - Min segment length:" << stats.minSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Raw stats - Max segment length:" << stats.maxSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Raw stats - Avg segment length:" << stats.averageSegmentLength << "mm";
    
    // Check if segment lengths are zero and warn
    if (stats.minSegmentLength == 0.0 && stats.maxSegmentLength == 0.0 && stats.averageSegmentLength == 0.0) {
        qDebug() << "[ConnectionWidget] WARNING: All segment lengths are zero! This indicates a problem with the solver.";
    }
    
    m_lastSolverStatistics.iterationsUsed = stats.iterationsUsed;
    m_lastSolverStatistics.finalDistanceToTarget = stats.finalDistanceToTarget;
    m_lastSolverStatistics.converged = stats.converged;
    m_lastSolverStatistics.simulationTime = stats.simulationTime;
    m_lastSolverStatistics.rigidBodiesCreated = stats.rigidBodiesCreated;
    m_lastSolverStatistics.jointsCreated = stats.jointsCreated;
    m_lastSolverStatistics.minSegmentLength = stats.minSegmentLength;
    m_lastSolverStatistics.maxSegmentLength = stats.maxSegmentLength;
    m_lastSolverStatistics.averageSegmentLength = stats.averageSegmentLength;
    
    qDebug() << "[ConnectionWidget] createDragChainPath: Stored solver statistics:";
    qDebug() << "[ConnectionWidget]   Stored - Distance to target:" << m_lastSolverStatistics.finalDistanceToTarget << "mm";
    qDebug() << "[ConnectionWidget]   Stored - Converged:" << m_lastSolverStatistics.converged;
    qDebug() << "[ConnectionWidget]   Stored - Iterations:" << m_lastSolverStatistics.iterationsUsed;
    qDebug() << "[ConnectionWidget]   Stored - Simulation time:" << m_lastSolverStatistics.simulationTime << "s";
    qDebug() << "[ConnectionWidget]   Stored - Min segment length:" << m_lastSolverStatistics.minSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Stored - Max segment length:" << m_lastSolverStatistics.maxSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Stored - Avg segment length:" << m_lastSolverStatistics.averageSegmentLength << "mm";
    
    // Update GUI labels immediately with solver statistics
    updateSolverStatisticsDisplay();
    
    // Force GUI update to ensure the statistics are displayed
    QApplication::processEvents();
    
    // Show completion status with enhanced information
    if (m_lastSolverStatistics.converged) {
        m_segmentLengthsLabel->setText(QString("Segment Lengths: Min=%1, Max=%2, Avg=%3 mm (✓ Solved)")
            .arg(m_lastSolverStatistics.minSegmentLength, 0, 'f', 1)
            .arg(m_lastSolverStatistics.maxSegmentLength, 0, 'f', 1)
            .arg(m_lastSolverStatistics.averageSegmentLength, 0, 'f', 1));
        m_solverIterationsLabel->setText(QString("Solver Iterations: %1/%2 (✓ Complete)")
            .arg(m_lastSolverStatistics.iterationsUsed)
            .arg(m_maxIterationsSpin->value()));
    } else {
        m_segmentLengthsLabel->setText(QString("Segment Lengths: Min=%1, Max=%2, Avg=%3 mm (⚠ Not Converged)")
            .arg(m_lastSolverStatistics.minSegmentLength, 0, 'f', 1)
            .arg(m_lastSolverStatistics.maxSegmentLength, 0, 'f', 1)
            .arg(m_lastSolverStatistics.averageSegmentLength, 0, 'f', 1));
        m_solverIterationsLabel->setText(QString("Solver Iterations: %1/%2 (⚠ Incomplete)")
            .arg(m_lastSolverStatistics.iterationsUsed)
            .arg(m_maxIterationsSpin->value()));
    }
    
    // Force another GUI update to ensure the final status is displayed
    QApplication::processEvents();
    
    // Final check to ensure statistics are displayed correctly
    qDebug() << "[ConnectionWidget] Final solver statistics after completion:";
    qDebug() << "[ConnectionWidget]   Iterations:" << m_lastSolverStatistics.iterationsUsed;
    qDebug() << "[ConnectionWidget]   Converged:" << m_lastSolverStatistics.converged;
    qDebug() << "[ConnectionWidget]   Min segment length:" << m_lastSolverStatistics.minSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Max segment length:" << m_lastSolverStatistics.maxSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Avg segment length:" << m_lastSolverStatistics.averageSegmentLength << "mm";
    
    // Convert solved segments to visualization format
    std::vector<ConnectionPathSegment> solverSegments = solver.convertToVisualizationSegments(
        solvedSegments,
        QVector4D(1, 0, 0, 0.8f),  // Main path color
        QVector4D(0, 1, 0, 1.0f)   // Indicator color
    );
        
    // Convert to CadOpenGLWidget::ConnectionPathSegment format
    std::vector<CadOpenGLWidget::ConnectionPathSegment> segments;
    for (const auto& solverSegment : solverSegments) {
        segments.emplace_back(solverSegment.start, solverSegment.end, solverSegment.color, solverSegment.width, solverSegment.isBend);
    }
    
    // Add attachment indicators if locked
    if (startLocked) {
        QVector3D startDir = getDirectionFromIndex(m_startDirectionCombo->currentIndex());
        QVector3D startIndicatorStart = point1 - startDir * (bendRadius * 0.2f);
        QVector3D startIndicatorEnd = point1 + startDir * (bendRadius * 0.2f);
        segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
            startIndicatorStart, startIndicatorEnd, QVector4D(0, 1, 0, 1.0f), 5.0f, true));
    }
    
    if (endLocked) {
        QVector3D endDir = getDirectionFromIndex(m_endDirectionCombo->currentIndex());
        QVector3D endIndicatorStart = point2 - endDir * (bendRadius * 0.2f);
        QVector3D endIndicatorEnd = point2 + endDir * (bendRadius * 0.2f);
        segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
            endIndicatorStart, endIndicatorEnd, QVector4D(0, 1, 0, 1.0f), 5.0f, true));
    }
    
    qDebug() << "[ConnectionWidget] Created constraint-based drag chain with" << segments.size() << "segments";
    qDebug() << "[ConnectionWidget] Solved segments:" << solvedSegments.size();
    
    return segments;
}

void ConnectionCreationWidget::updateSolverStatisticsDisplay()
{
    qDebug() << "[ConnectionWidget] updateSolverStatisticsDisplay: Updating GUI with solver statistics";
    
    // Check if we have valid solver statistics (either from a completed solver or from stored results)
    if (hasValidSolverStatistics()) {
        qDebug() << "[ConnectionWidget] Found stored solver statistics:";
        qDebug() << "[ConnectionWidget]   Min segment length:" << m_lastSolverStatistics.minSegmentLength << "mm";
        qDebug() << "[ConnectionWidget]   Max segment length:" << m_lastSolverStatistics.maxSegmentLength << "mm";
        qDebug() << "[ConnectionWidget]   Avg segment length:" << m_lastSolverStatistics.averageSegmentLength << "mm";
        qDebug() << "[ConnectionWidget]   Converged:" << m_lastSolverStatistics.converged;
        qDebug() << "[ConnectionWidget]   Iterations:" << m_lastSolverStatistics.iterationsUsed;
        
        // Use stored statistics for display with enhanced information
        QString segmentLengthsText = QString("Segment Lengths: Min=%1, Max=%2, Avg=%3 mm (%4)")
            .arg(m_lastSolverStatistics.minSegmentLength, 0, 'f', 1)
            .arg(m_lastSolverStatistics.maxSegmentLength, 0, 'f', 1)
            .arg(m_lastSolverStatistics.averageSegmentLength, 0, 'f', 1)
            .arg(m_lastSolverStatistics.converged ? "✓ Converged" : "✗ Not Converged");
        m_segmentLengthsLabel->setText(segmentLengthsText);
        
        QString iterationsText = QString("Solver Iterations: %1/%2 (%.3f s) [%3 bodies, %4 joints]")
            .arg(m_lastSolverStatistics.iterationsUsed)
            .arg(m_maxIterationsSpin->value())
            .arg(m_lastSolverStatistics.simulationTime, 0, 'f', 3)
            .arg(m_lastSolverStatistics.rigidBodiesCreated)
            .arg(m_lastSolverStatistics.jointsCreated);
        m_solverIterationsLabel->setText(iterationsText);
        
        qDebug() << "[ConnectionWidget] Updated GUI labels from stored statistics:";
        qDebug() << "[ConnectionWidget]   Segment lengths label:" << segmentLengthsText;
        qDebug() << "[ConnectionWidget]   Iterations label:" << iterationsText;
        
        // Force GUI update to ensure labels are displayed
        QApplication::processEvents();
    } else {
        qDebug() << "[ConnectionWidget] No stored solver statistics available";
        m_segmentLengthsLabel->setText("Segment Lengths: --");
        m_solverIterationsLabel->setText("Solver Iterations: --");
        
        // Force GUI update to ensure labels are displayed
        QApplication::processEvents();
    }
}

void ConnectionCreationWidget::updateConnectionInfo()
{
    qDebug() << "[ConnectionWidget] updateConnectionInfo: Called";
    if (!m_point1 || !m_point2) {
        m_distanceLabel->setText("Distance: --");
        m_calculatedLengthLabel->setText("Calculated Length: --");
        m_effectiveLengthLabel->setText("Effective Length: --");
        m_compatibilityLabel->setText("Compatibility: --");
        m_placementLabel->setText("Placement: --");
        m_segmentLengthsLabel->setText("Segment Lengths: --");
        m_solverIterationsLabel->setText("Solver Iterations: --");
        m_createButton->setEnabled(false);
        return;
    }
    
    // Check compatibility
    bool compatible = canConnectPoints(m_point1.get(), m_point2.get());
    m_compatibilityLabel->setText(compatible ? "Compatibility: ✓ Compatible" : "Compatibility: ✗ Incompatible");
    
    // Calculate distance
    double distance = calculateDistanceBetweenPoints(m_point1.get(), m_point2.get());
    m_distanceLabel->setText(QString("Distance: %1 mm").arg(distance, 0, 'f', 2));
    
    // Calculate lengths using constraint solver results
    double calculatedLength = distance;
    double effectiveLength;
    int totalSegments = 0;
    int baseSegments = 0;
    
    // Check if this is a drag chain with locked attachments
    bool isDragChain = (m_connectionTypeCombo->currentData().toInt() == static_cast<int>(ConnectionNodeData::ConnectionType::DragChain));
    bool startLocked = m_lockStartAttachmentCheck->isChecked();
    bool endLocked = m_lockEndAttachmentCheck->isChecked();
    
    if (isDragChain) {
        qDebug() << "[ConnectionWidget] updateConnectionInfo: Using drag chain solver for statistics";
        
        // For drag chains, we only display statistics - the solver is called by updateConnectionPathVisualization
        qDebug() << "[ConnectionWidget] updateConnectionInfo: Checking solver statistics - iterationsUsed:" << m_lastSolverStatistics.iterationsUsed;
        qDebug() << "[ConnectionWidget] updateConnectionInfo: converged:" << m_lastSolverStatistics.converged;
        qDebug() << "[ConnectionWidget] updateConnectionInfo: minSegmentLength:" << m_lastSolverStatistics.minSegmentLength;
        qDebug() << "[ConnectionWidget] updateConnectionInfo: maxSegmentLength:" << m_lastSolverStatistics.maxSegmentLength;
        
        // Check if we have valid solver statistics (either from a completed solver or from stored results)
        if (hasValidSolverStatistics()) {
            qDebug() << "[ConnectionWidget] updateConnectionInfo: Using existing solver statistics";
            updateSolverStatisticsDisplay();
            
            // Calculate effective length based on stored statistics
            effectiveLength = distance; // Use basic distance for now
            totalSegments = m_segmentCountSpin->value();
            baseSegments = static_cast<int>(std::ceil(distance / m_pitchLengthSpin->value()));
        } else {
            qDebug() << "[ConnectionWidget] updateConnectionInfo: No solver statistics available yet";
            // Don't call the solver here - let updateConnectionPathVisualization handle it
            
            // Use fallback calculation
            effectiveLength = distance;
            totalSegments = m_segmentCountSpin->value();
            baseSegments = static_cast<int>(std::ceil(distance / m_pitchLengthSpin->value()));
        }
            } else {
            // Use standard calculation with pitch length and segments
            double pitchLength = m_pitchLengthSpin->value();
            int segmentCount = m_segmentCountSpin->value();
            
            // Clear solver information for non-solver calculations
            m_segmentLengthsLabel->setText("Segment Lengths: -- (Non-Drag Chain)");
            m_solverIterationsLabel->setText("Solver Iterations: -- (Not Applicable)");
        
        // Calculate base length using pitch length
        baseSegments = static_cast<int>(std::ceil(distance / pitchLength));
        effectiveLength = segmentCount * pitchLength;
        
        // Add bend radius contribution
        if (m_maxBendRadiusSpin->value() > 0.0) {
            double typicalSegmentLength = pitchLength;
            int estimatedBends = std::max(1, static_cast<int>(distance / typicalSegmentLength));
            double bendLength = estimatedBends * (m_maxBendRadiusSpin->value() * M_PI / 2.0);
            double transitionLength = estimatedBends * 20.0;
            effectiveLength += bendLength + transitionLength;
        }
        
        totalSegments = segmentCount;
    }
    
    m_calculatedLengthLabel->setText(QString("Calculated Length: %1 mm (%2 segments)").arg(calculatedLength, 0, 'f', 2).arg(baseSegments));
    m_effectiveLengthLabel->setText(QString("Effective Length: %1 mm (%2 segments)").arg(effectiveLength, 0, 'f', 2).arg(totalSegments));
    
    // Show placement location
    CadNode* placement = findBestConnectionPlacement(m_point1.get(), m_point2.get());
    if (placement) {
        m_placementLabel->setText(QString("Placement: %1").arg(QString::fromStdString(placement->name)));
    } else {
        m_placementLabel->setText("Placement: --");
    }
    
    // Enable create button if valid
    m_createButton->setEnabled(compatible && !m_connectionNameEdit->text().trimmed().isEmpty());
}

void ConnectionCreationWidget::onCreateConnection()
{
    if (!validateConnection()) {
        return;
    }
    
    // Get connection type
    ConnectionNodeData::ConnectionType connectionType = static_cast<ConnectionNodeData::ConnectionType>(
        m_connectionTypeCombo->currentData().toInt()
    );
    
    // For drag chains, ensure the solver has been called to get statistics
    if (connectionType == ConnectionNodeData::ConnectionType::DragChain) {
        qDebug() << "[ConnectionWidget] onCreateConnection: Ensuring drag chain solver has been called";
        
        // Update connection info to trigger solver if not already done
        updateConnectionInfo();
        
        // Update visualization to ensure solver statistics are available
        updateConnectionPathVisualization();
        
        qDebug() << "[ConnectionWidget] onCreateConnection: Solver statistics after update:";
        qDebug() << "[ConnectionWidget]   iterationsUsed:" << m_lastSolverStatistics.iterationsUsed;
        qDebug() << "[ConnectionWidget]   finalDistanceToTarget:" << m_lastSolverStatistics.finalDistanceToTarget;
        qDebug() << "[ConnectionWidget]   converged:" << m_lastSolverStatistics.converged;
    }
    
    // Create the connection
    m_connectionNode = createConnectionBetweenPoints(
        m_connectionNameEdit->text().toStdString(),
        m_point1.get(), m_point2.get(),
        connectionType
    );
    
    if (m_connectionNode) {
        // Set the connection data
        auto connectionData = std::dynamic_pointer_cast<ConnectionNodeData>(m_connectionNode->data);
        if (connectionData) {
            connectionData->isFlexible = m_isFlexibleCheck->isChecked();
            connectionData->maxBendRadius = m_maxBendRadiusSpin->value();
            connectionData->pitchLength = m_pitchLengthSpin->value();
            connectionData->segmentCount = m_segmentCountSpin->value();
            
            // Set attachment configurations for drag chains
            if (connectionData->connectionType == ConnectionNodeData::ConnectionType::DragChain) {
                // Start attachment configuration
                connectionData->startAttachment.isLocked = m_lockStartAttachmentCheck->isChecked();
                connectionData->startAttachment.planeNormal = getPlaneNormalFromIndex(m_startPlaneCombo->currentIndex());
                connectionData->startAttachment.direction = getDirectionFromIndex(m_startDirectionCombo->currentIndex());
                
                // End attachment configuration
                connectionData->endAttachment.isLocked = m_lockEndAttachmentCheck->isChecked();
                connectionData->endAttachment.planeNormal = getPlaneNormalFromIndex(m_endPlaneCombo->currentIndex());
                connectionData->endAttachment.direction = getDirectionFromIndex(m_endDirectionCombo->currentIndex());
            }
        }
        
        // Emit signal
        emit connectionCreated(m_connectionNode);
        
        // Clear the widget state
        clearState();
    }
}

void ConnectionCreationWidget::onCancel()
{
    emit connectionCancelled();
    clearState();
}

void ConnectionCreationWidget::onSolverStatisticsUpdated(const DragChainConstraintSolver::SolverStatistics& stats)
{
    qDebug() << "[ConnectionWidget] onSolverStatisticsUpdated: Real-time statistics update received";
    qDebug() << "[ConnectionWidget]   Min segment length:" << stats.minSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Max segment length:" << stats.maxSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Avg segment length:" << stats.averageSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Converged:" << stats.converged;
    qDebug() << "[ConnectionWidget]   Iterations:" << stats.iterationsUsed;
    qDebug() << "[ConnectionWidget]   Simulation time:" << stats.simulationTime << "s";
    
    // Update stored statistics
    m_lastSolverStatistics.iterationsUsed = stats.iterationsUsed;
    m_lastSolverStatistics.finalDistanceToTarget = stats.finalDistanceToTarget;
    m_lastSolverStatistics.converged = stats.converged;
    m_lastSolverStatistics.simulationTime = stats.simulationTime;
    m_lastSolverStatistics.rigidBodiesCreated = stats.rigidBodiesCreated;
    m_lastSolverStatistics.jointsCreated = stats.jointsCreated;
    m_lastSolverStatistics.minSegmentLength = stats.minSegmentLength;
    m_lastSolverStatistics.maxSegmentLength = stats.maxSegmentLength;
    m_lastSolverStatistics.averageSegmentLength = stats.averageSegmentLength;
    
    // Update GUI immediately with real-time statistics
    updateSolverStatisticsDisplay();
    
    // Force GUI update to ensure real-time display
    QApplication::processEvents();
}

bool ConnectionCreationWidget::validateConnection()
{
    if (!m_point1 || !m_point2) {
        QMessageBox::warning(this, "Validation Error", "Please select both connection points.");
        return false;
    }
    
    if (m_point1 == m_point2) {
        QMessageBox::warning(this, "Validation Error", "Cannot connect a point to itself.");
        return false;
    }
    
    if (!canConnectPoints(m_point1.get(), m_point2.get())) {
        QMessageBox::warning(this, "Validation Error", "The selected points are not compatible for connection.");
        return false;
    }
    
    if (m_connectionNameEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "Validation Error", "Please enter a name for the connection.");
        return false;
    }
    
    return true;
}

void ConnectionCreationWidget::clearState()
{
    m_connectionNode.reset();
    m_point1.reset();
    m_point2.reset();
    
    m_point1List->setCurrentRow(-1);
    m_point2List->setCurrentRow(-1);
    m_connectionNameEdit->setText("New Connection");
    m_connectionTypeCombo->setCurrentIndex(0);
    m_isFlexibleCheck->setChecked(true);
    m_maxBendRadiusSpin->setValue(50.0);
    m_pitchLengthSpin->setValue(300.0);
    m_segmentCountSpin->setValue(3);
    
    // Reset attachment configuration UI
    m_lockStartAttachmentCheck->setChecked(false);
    m_lockEndAttachmentCheck->setChecked(false);
    m_startPlaneCombo->setCurrentIndex(0);
    m_endPlaneCombo->setCurrentIndex(0);
    m_startDirectionCombo->setCurrentIndex(0);
    m_endDirectionCombo->setCurrentIndex(0);
    
    updateConnectionInfo();
    updateConnectionPathVisualization();
} 

void ConnectionCreationWidget::setOpenGLWidget(CadOpenGLWidget* openGLWidget)
{
    qDebug() << "[ConnectionWidget] setOpenGLWidget called with:" << (openGLWidget ? "valid" : "null");
    m_openGLWidget = openGLWidget;
    if (m_openGLWidget) {
        qDebug() << "[ConnectionWidget] OpenGL widget set successfully";
        qDebug() << "[ConnectionWidget] Root node:" << (m_openGLWidget->getRootTreeNode() ? "valid" : "null");
    }
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::updateConnectionPathVisualization()
{
    qDebug() << "[ConnectionWidget] updateConnectionPathVisualization called";
    qDebug() << "[ConnectionWidget] m_openGLWidget:" << (m_openGLWidget ? "valid" : "null");
    qDebug() << "[ConnectionWidget] m_point1:" << (m_point1 ? "valid" : "null");
    qDebug() << "[ConnectionWidget] m_point2:" << (m_point2 ? "valid" : "null");
    
    if (!m_openGLWidget) {
        qDebug() << "[ConnectionWidget] No OpenGL widget available";
        return;
    }
    
    if (!m_point1 || !m_point2) {
        qDebug() << "[ConnectionWidget] One or both points not selected, clearing segments";
            m_openGLWidget->clearConnectionPathSegments();
        return;
    }
    
    qDebug() << "[ConnectionWidget] Both points selected, calculating positions...";
    
    // Get the root node from the OpenGL widget
    CadNode* rootNode = m_openGLWidget->getRootTreeNode();
    if (!rootNode) {
        qDebug() << "[ConnectionWidget] No root node available";
        return;
    }
    
    qDebug() << "[ConnectionWidget] Root node found:" << QString::fromStdString(rootNode->name);
    
    // Get simulation manager from OpenGL widget
    SimulationManager* simManager = m_openGLWidget->getSimulationManager();
    qDebug() << "[ConnectionWidget] Simulation manager:" << (simManager ? "valid" : "null");
    
    // Helper function to get node location (same as in CadOpenGLWidget)
    auto getNodeLocation = [simManager](const CadNode* node) -> TopLoc_Location {
        if (!node) return TopLoc_Location();
        
        // If we have a simulation manager, check for updated locations
        if (simManager && simManager->hasNodeUpdates()) {
            const auto& nodeLocations = simManager->getLatestNodeLocations();
            auto it = nodeLocations.find(const_cast<CadNode*>(node));
            if (it != nodeLocations.end()) {
                return it->second;
            }
        }
        
        // Fall back to the node's original location
        return node->loc;
    };
    
    // Helper function to find accumulated location for a node (same logic as traverseAndRender)
    std::function<TopLoc_Location(CadNode*, CadNode*, TopLoc_Location)> findAccumulatedLocation = 
        [&](CadNode* target, CadNode* current, TopLoc_Location parentAccum) -> TopLoc_Location {
            if (!current) return TopLoc_Location();
            
            // Get the node location (same as in traverseAndRender)
            TopLoc_Location nodeLoc = getNodeLocation(current);
            
            // Always accumulate the transform for this node (same as in traverseAndRender)
            TopLoc_Location newAccumulatedLoc = parentAccum * nodeLoc;
            
            // If this is our target, return the accumulated location
            if (current == target) {
                qDebug() << "[ConnectionWidget] Found target node:" << QString::fromStdString(current->name);
                return newAccumulatedLoc;
            }
            
            // Recursively search children
            for (const auto& child : current->children) {
                if (child) {
                    TopLoc_Location childLoc = findAccumulatedLocation(target, child.get(), newAccumulatedLoc);
                    if (!childLoc.IsIdentity()) {
                        // Found the target in this child branch
                        return childLoc;
                    }
                }
            }
            
            return TopLoc_Location();
        };
    
    // Find the original nodes in the tree for location calculation
    int point1Index = m_point1List->currentRow();
    int point2Index = m_point2List->currentRow();
    
    if (point1Index < 0 || point1Index >= static_cast<int>(m_originalConnectionPoints.size()) ||
        point2Index < 0 || point2Index >= static_cast<int>(m_originalConnectionPoints.size())) {
        qDebug() << "[ConnectionWidget] Invalid point indices:" << point1Index << point2Index;
        m_openGLWidget->clearConnectionPathSegments();
        return;
    }
    
    CadNode* originalPoint1 = m_originalConnectionPoints[point1Index];
    CadNode* originalPoint2 = m_originalConnectionPoints[point2Index];
    
    // Find accumulated locations for both points (starting from root with identity transform)
    qDebug() << "[ConnectionWidget] Finding accumulated location for point1:" << QString::fromStdString(originalPoint1->name);
    qDebug() << "[ConnectionWidget] Original Point1 address:" << static_cast<const void*>(originalPoint1);
    qDebug() << "[ConnectionWidget] Root node address:" << static_cast<const void*>(rootNode);
    TopLoc_Location accumulatedLoc1 = findAccumulatedLocation(originalPoint1, rootNode, TopLoc_Location());
    qDebug() << "[ConnectionWidget] Point1 accumulated location identity:" << accumulatedLoc1.IsIdentity();
    
    qDebug() << "[ConnectionWidget] Finding accumulated location for point2:" << QString::fromStdString(originalPoint2->name);
    qDebug() << "[ConnectionWidget] Original Point2 address:" << static_cast<const void*>(originalPoint2);
    TopLoc_Location accumulatedLoc2 = findAccumulatedLocation(originalPoint2, rootNode, TopLoc_Location());
    qDebug() << "[ConnectionWidget] Point2 accumulated location identity:" << accumulatedLoc2.IsIdentity();
    
    if (accumulatedLoc1.IsIdentity() || accumulatedLoc2.IsIdentity()) {
        qDebug() << "[ConnectionWidget] Could not find accumulated locations for points";
        qDebug() << "[ConnectionWidget] Point1 identity:" << accumulatedLoc1.IsIdentity();
        qDebug() << "[ConnectionWidget] Point2 identity:" << accumulatedLoc2.IsIdentity();
        qDebug() << "[ConnectionWidget] Root node name:" << QString::fromStdString(rootNode->name);
        qDebug() << "[ConnectionWidget] Point1 name:" << QString::fromStdString(m_point1->name);
        qDebug() << "[ConnectionWidget] Point2 name:" << QString::fromStdString(m_point2->name);
        m_openGLWidget->clearConnectionPathSegments();
        return;
    }
    
    // Get world positions using the accumulated transforms
    gp_Pnt worldPos1 = accumulatedLoc1.Transformation().TranslationPart();
    gp_Pnt worldPos2 = accumulatedLoc2.Transformation().TranslationPart();
    
    QVector3D point1(worldPos1.X(), worldPos1.Y(), worldPos1.Z());
    QVector3D point2(worldPos2.X(), worldPos2.Y(), worldPos2.Z());
    
    qDebug() << "[ConnectionWidget] Point1 world position:" << point1;
    qDebug() << "[ConnectionWidget] Point2 world position:" << point2;
    
    // Check if the coordinates are reasonable (not too large or small)
    double maxCoord = std::max({std::abs(point1.x()), std::abs(point1.y()), std::abs(point1.z()),
                               std::abs(point2.x()), std::abs(point2.y()), std::abs(point2.z())});
    qDebug() << "[ConnectionWidget] Maximum coordinate value:" << maxCoord;
    
    if (maxCoord > 10000.0) {
        qDebug() << "[ConnectionWidget] WARNING: Coordinates are very large, scaling down for visualization";
        point1 *= 0.001f; // Scale down by 1000
        point2 *= 0.001f;
        qDebug() << "[ConnectionWidget] Scaled Point1:" << point1;
        qDebug() << "[ConnectionWidget] Scaled Point2:" << point2;
    } else if (maxCoord < 0.1) {
        qDebug() << "[ConnectionWidget] WARNING: Coordinates are very small, scaling up for visualization";
        point1 *= 1000.0f; // Scale up by 1000
        point2 *= 1000.0f;
        qDebug() << "[ConnectionWidget] Scaled Point1:" << point1;
        qDebug() << "[ConnectionWidget] Scaled Point2:" << point2;
    }
    
    // Calculate the distance and direction
    QVector3D direction = point2 - point1;
    double distance = direction.length();
    direction.normalize();
    
    qDebug() << "[ConnectionWidget] Distance between points:" << distance;
    
    // Create path segments based on connection type and parameters
    std::vector<CadOpenGLWidget::ConnectionPathSegment> segments;
    
    // Get connection parameters
    double bendRadius = m_maxBendRadiusSpin->value();
    double pitchLength = m_pitchLengthSpin->value();
    int segmentCount = m_segmentCountSpin->value();
    bool isFlexible = m_isFlexibleCheck->isChecked();
    bool isDragChain = (m_connectionTypeCombo->currentData().toInt() == static_cast<int>(ConnectionNodeData::ConnectionType::DragChain));
    bool startLocked = m_lockStartAttachmentCheck->isChecked();
    bool endLocked = m_lockEndAttachmentCheck->isChecked();
    
    qDebug() << "[ConnectionWidget] Connection parameters - bendRadius:" << bendRadius << "pitchLength:" << pitchLength << "segmentCount:" << segmentCount << "isFlexible:" << isFlexible;
    qDebug() << "[ConnectionWidget] Drag chain settings - isDragChain:" << isDragChain << "startLocked:" << startLocked << "endLocked:" << endLocked;
    
    if (isFlexible && bendRadius > 0.0) {
        if (isDragChain) {
            qDebug() << "[ConnectionWidget] Creating unified drag chain path";
            qDebug() << "[ConnectionWidget]   startLocked:" << startLocked << "endLocked:" << endLocked;
            
            // Always use solver for drag chains to enforce constraints properly
            qDebug() << "[ConnectionWidget]   Will call solver (drag chain requires constraint enforcement)";
            segments = createDragChainPath(point1, point2, direction, distance, bendRadius, pitchLength, segmentCount, startLocked, endLocked);
        } else {
            qDebug() << "[ConnectionWidget] Creating simple path with one bend for non-drag chain";
            // Create a simple path with one bend for non-drag chain connections
            QVector3D midPoint = (point1 + point2) * 0.5f;
            
            // Add some offset perpendicular to the direction for the bend
            QVector3D perpendicular;
            if (std::abs(direction.x()) < 0.9f) {
                perpendicular = QVector3D::crossProduct(QVector3D(1, 0, 0), direction);
            } else {
                perpendicular = QVector3D::crossProduct(QVector3D(0, 1, 0), direction);
            }
            perpendicular.normalize();
            
            // Create bend offset
            QVector3D bendOffset = perpendicular * (bendRadius * 0.3f);
            QVector3D bendPoint = midPoint + bendOffset;
            
            // Create segments: point1 -> bendPoint -> point2
            segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
                point1, bendPoint, QVector4D(1, 0, 0, 0.8f), 3.0f, false));
            segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
                bendPoint, point2, QVector4D(1, 0, 0, 0.8f), 3.0f, false));
            
            // Add bend indicator
            segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
                bendPoint - bendOffset * 0.5f, bendPoint + bendOffset * 0.5f, 
                QVector4D(0, 1, 0, 1.0f), 5.0f, true));
        }
    } else {
        qDebug() << "[ConnectionWidget] Creating simple straight line for rigid connections";
        // Simple straight line for rigid connections
        segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
            point1, point2, QVector4D(0, 0, 1, 0.8f), 3.0f, false));
        
        // Clear solver statistics for rigid connection
        clearSolverStatistics();
        qDebug() << "[ConnectionWidget]   Cleared solver statistics for rigid connection";
    }
    
    // Set the segments in the OpenGL widget
    qDebug() << "[ConnectionWidget] Created" << segments.size() << "segments";
    for (size_t i = 0; i < segments.size(); ++i) {
        qDebug() << "[ConnectionWidget] Segment" << i << ":" << segments[i].start << "->" << segments[i].end;
    }
    
    qDebug() << "[ConnectionWidget] About to call setConnectionPathSegments with" << segments.size() << "segments";
    m_openGLWidget->setConnectionPathSegments(segments);
    qDebug() << "[ConnectionWidget] setConnectionPathSegments call completed";
    
    // Update connection info to display solver statistics
    qDebug() << "[ConnectionWidget] updateConnectionPathVisualization: About to call updateConnectionInfo with iterationsUsed:" << m_lastSolverStatistics.iterationsUsed;
    qDebug() << "[ConnectionWidget] updateConnectionPathVisualization: converged:" << m_lastSolverStatistics.converged;
    qDebug() << "[ConnectionWidget] updateConnectionPathVisualization: minSegmentLength:" << m_lastSolverStatistics.minSegmentLength;
    qDebug() << "[ConnectionWidget] updateConnectionPathVisualization: maxSegmentLength:" << m_lastSolverStatistics.maxSegmentLength;
    
    // Only update connection info if we don't have valid solver statistics to preserve them
    if (!hasValidSolverStatistics()) {
        updateConnectionInfo();
    } else {
        qDebug() << "[ConnectionWidget] Preserving solver statistics, skipping updateConnectionInfo";
    }
    
    qDebug() << "[ConnectionWidget] updateConnectionPathVisualization completed - solver statistics preserved";
}

bool ConnectionCreationWidget::hasValidSolverStatistics() const
{
    bool hasValid = (m_lastSolverStatistics.iterationsUsed > 0) || 
                    (m_lastSolverStatistics.minSegmentLength > 0.0 || 
                     m_lastSolverStatistics.maxSegmentLength > 0.0 || 
                     m_lastSolverStatistics.averageSegmentLength > 0.0);
    
    qDebug() << "[ConnectionWidget] hasValidSolverStatistics: " << (hasValid ? "true" : "false")
             << " (iterations:" << m_lastSolverStatistics.iterationsUsed
             << ", minLength:" << m_lastSolverStatistics.minSegmentLength
             << ", maxLength:" << m_lastSolverStatistics.maxSegmentLength
             << ", avgLength:" << m_lastSolverStatistics.averageSegmentLength << ")";
    
    return hasValid;
} 