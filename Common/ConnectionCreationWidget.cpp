#include "ConnectionCreationWidget.h"
#include "HelperFunctions.h"
#include "CadOpenGLWidget.h"
#include "SimulationManager.h"
#include "BezierDragChainSolver.h"
#include <QApplication>
#include <QScreen>
#include <QVector3D>
#include <cmath>
#include <QPainter>
#include <QMouseEvent>
#include <QPainterPath>



ConnectionCreationWidget::ConnectionCreationWidget(CustomModelTreeModel* model, QWidget* parent)
    : QWidget(parent)
    , m_model(model)
    , m_point1(nullptr)
    , m_point2(nullptr)
    , m_userModifiedControlPoint(false)
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
    qDebug() << "[ConnectionWidget] Setup: Drag Chain enum value:" << static_cast<int>(ConnectionNodeData::ConnectionType::DragChain);
    qDebug() << "[ConnectionWidget] Setup: Initial connection type index:" << m_connectionTypeCombo->currentIndex();
    qDebug() << "[ConnectionWidget] Setup: Initial connection type data:" << m_connectionTypeCombo->currentData().toInt();
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
    m_pitchLengthSpin->setToolTip("Length of each drag chain segment (segment count is calculated automatically)");
    propertiesLayout->addRow("Pitch Length:", m_pitchLengthSpin);
    
    // Bézier curve solver parameters
    m_optimizationStepSpin = new QDoubleSpinBox();
    m_optimizationStepSpin->setRange(0.01, 1.0);
    m_optimizationStepSpin->setValue(0.1);
    m_optimizationStepSpin->setSuffix(" step size");
    m_optimizationStepSpin->setDecimals(2);
    m_optimizationStepSpin->setToolTip("Optimization step size for control point adjustment");
    propertiesLayout->addRow("Optimization Step:", m_optimizationStepSpin);
    
    m_curveSmoothnessSpin = new QDoubleSpinBox();
    m_curveSmoothnessSpin->setRange(0.1, 10.0);
    m_curveSmoothnessSpin->setValue(1.0);
    m_curveSmoothnessSpin->setSuffix(" smoothness");
    m_curveSmoothnessSpin->setDecimals(1);
    m_curveSmoothnessSpin->setToolTip("Curve smoothness factor (higher = smoother)");
    propertiesLayout->addRow("Curve Smoothness:", m_curveSmoothnessSpin);
    
    // Deviation threshold parameter
    m_deviationThresholdSpin = new QDoubleSpinBox();
    m_deviationThresholdSpin->setRange(0.1, 100.0);
    m_deviationThresholdSpin->setValue(1.8); // Default to 10% of typical pitch length
    m_deviationThresholdSpin->setSuffix(" mm");
    m_deviationThresholdSpin->setDecimals(1);
    m_deviationThresholdSpin->setToolTip("Distance threshold for flagging segments as deviating from the intended path");
    propertiesLayout->addRow("Deviation Threshold:", m_deviationThresholdSpin);
    
    // Drag chain cross-sectional dimensions
    m_chainWidthSpin = new QDoubleSpinBox();
    m_chainWidthSpin->setRange(1.0, 100.0);
    m_chainWidthSpin->setValue(20.0);
    m_chainWidthSpin->setSuffix(" mm");
    m_chainWidthSpin->setDecimals(1);
    m_chainWidthSpin->setToolTip("Width of the drag chain cross-section for 3D visualization");
    propertiesLayout->addRow("Chain Width:", m_chainWidthSpin);
    
    m_chainHeightSpin = new QDoubleSpinBox();
    m_chainHeightSpin->setRange(1.0, 500.0);
    m_chainHeightSpin->setValue(20.0);
    m_chainHeightSpin->setSuffix(" mm");
    m_chainHeightSpin->setDecimals(1);
    m_chainHeightSpin->setToolTip("Height of the drag chain cross-section for 3D visualization");
    propertiesLayout->addRow("Chain Height:", m_chainHeightSpin);
    
    // Segment count is now always calculated automatically based on pitch length
    // Enhanced visualization, strict angle constraints, and path recovery are always enabled
    
    mainLayout->addWidget(propertiesGroup);
    
    // 2D Visualization
    auto visualizationGroup = new QGroupBox("2D Path Visualization");
    auto visualizationLayout = new QVBoxLayout(visualizationGroup);
    
    m_2dVisualization = new DragChain2DVisualization();
    m_2dVisualization->setMinimumHeight(300);
    m_2dVisualization->setMaximumHeight(400);
    m_2dVisualization->setToolTip("2D visualization of the drag chain path. Click and drag to move control points. Double-click to add new control points. Select a control point and press Delete to remove it.");
    visualizationLayout->addWidget(m_2dVisualization);
    
    // Add instruction label
    auto instructionLabel = new QLabel("Controls: Click and drag to move points. Double-click to add control points. Select and press Delete to remove.");
    instructionLabel->setStyleSheet("QLabel { color: gray; font-size: 10px; }");
    instructionLabel->setAlignment(Qt::AlignCenter);
    visualizationLayout->addWidget(instructionLabel);
    
    // Ensure 2D visualization can receive focus for keyboard events
    m_2dVisualization->setFocus();
    
    mainLayout->addWidget(visualizationGroup);
    
    // Connect 2D visualization signals
    connect(m_2dVisualization, &DragChain2DVisualization::startPointChanged, this, &ConnectionCreationWidget::onStartPointChanged);
    connect(m_2dVisualization, &DragChain2DVisualization::endPointChanged, this, &ConnectionCreationWidget::onEndPointChanged);
    connect(m_2dVisualization, &DragChain2DVisualization::controlPointChanged, this, &ConnectionCreationWidget::onControlPointChanged);
    connect(m_2dVisualization, &DragChain2DVisualization::controlPointAdded, this, &ConnectionCreationWidget::onControlPointAdded);
    connect(m_2dVisualization, &DragChain2DVisualization::controlPointRemoved, this, &ConnectionCreationWidget::onControlPointRemoved);
    
    // Sync control points from main widget to 2D visualization
    if (!m_currentControlPoints.empty()) {
        m_2dVisualization->setControlPoints(m_currentControlPoints);
    }
    
    // Connect deviation threshold changes
    connect(m_deviationThresholdSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &ConnectionCreationWidget::onDeviationThresholdChanged);
    
    // Connect chain dimension changes
    connect(m_chainWidthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &ConnectionCreationWidget::onChainDimensionsChanged);
    connect(m_chainHeightSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &ConnectionCreationWidget::onChainDimensionsChanged);
    
    mainLayout->addWidget(propertiesGroup);
    
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
    
    m_cableLengthLabel = new QLabel("Cable Length: --");
    infoLayout->addRow("Cable Length:", m_cableLengthLabel);
    
    m_segmentCountLabel = new QLabel("Segment Count: --");
    infoLayout->addRow("Segment Count:", m_segmentCountLabel);
    
    m_segmentAnglesLabel = new QLabel("Segment Angles: --");
    infoLayout->addRow("Segment Angles:", m_segmentAnglesLabel);
    
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
    connect(m_optimizationStepSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ConnectionCreationWidget::onConnectionTypeChanged);
    connect(m_curveSmoothnessSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ConnectionCreationWidget::onConnectionTypeChanged);
    
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
    
    // Reset user modification flag when connection type changes
    m_userModifiedControlPoint = false;
    // Clear current control points to force fresh auto-calculation
    m_currentControlPoints.clear();
    
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

void ConnectionCreationWidget::onUpdateVisualization()
{
    qDebug() << "[ConnectionWidget] Manual visualization update requested";
    updateConnectionPathVisualization();
}



QVector3D ConnectionCreationWidget::projectTo2DPlane(const QVector3D& point, const QVector3D& startPoint, const QVector3D& endPoint) const
{
    // Define the 2D plane using the start and end points
    QVector3D direction = (endPoint - startPoint).normalized();
    
    // Calculate the plane normal (perpendicular to the direction)
    QVector3D planeNormal;
    if (std::abs(direction.x()) < 0.9f) {
        planeNormal = QVector3D::crossProduct(QVector3D(1, 0, 0), direction);
    } else {
        planeNormal = QVector3D::crossProduct(QVector3D(0, 1, 0), direction);
    }
    planeNormal.normalize();
    
    // Project the point onto the plane
    // Formula: projected = point - (point - startPoint) · planeNormal * planeNormal
    QVector3D pointToStart = point - startPoint;
    double distanceFromPlane = QVector3D::dotProduct(pointToStart, planeNormal);
    QVector3D projectedPoint = point - planeNormal * distanceFromPlane;
    
    return projectedPoint;
}

std::vector<CadOpenGLWidget::ConnectionPathSegment> ConnectionCreationWidget::createDragChainPath(
    const QVector3D& point1, const QVector3D& point2, const QVector3D& direction,
    double distance, double bendRadius, double pitchLength)
{
    qDebug() << "[ConnectionWidget] createDragChainPath called with:";
    qDebug() << "[ConnectionWidget]   point1:" << point1;
    qDebug() << "[ConnectionWidget]   point2:" << point2;
    qDebug() << "[ConnectionWidget]   direction:" << direction;
    qDebug() << "[ConnectionWidget]   distance:" << distance;
    qDebug() << "[ConnectionWidget]   bendRadius:" << bendRadius;
    qDebug() << "[ConnectionWidget]   pitchLength:" << pitchLength;
    qDebug() << "[ConnectionWidget]   m_currentControlPoints.size():" << m_currentControlPoints.size();
    for (size_t i = 0; i < m_currentControlPoints.size(); ++i) {
        qDebug() << "[ConnectionWidget]     Control point" << i << ":" << m_currentControlPoints[i];
    }
    
    qDebug() << "[ConnectionWidget] Creating constraint-based drag chain path";
    qDebug() << "[ConnectionWidget] Parameters - distance:" << distance << "pitchLength:" << pitchLength;
    qDebug() << "[ConnectionWidget] User modified control point:" << (m_userModifiedControlPoint ? "true" : "false");
    
    // If user modified control points, clear solver statistics to force re-run
    if (m_userModifiedControlPoint) {
        qDebug() << "[ConnectionWidget] User modified control points detected, forcing solver re-run";
        clearSolverStatistics();
        m_userModifiedControlPoint = false; // Reset the flag
    }
    
    // Create Bézier curve solver instance
    BezierDragChainSolver solver;
    
    // Set solver parameters from GUI
    solver.setMaxIterations(50); // Fixed reasonable value for Bézier solver
    solver.setConvergenceTolerance(0.01); // Fixed reasonable value for Bézier solver
    solver.setBendRadiusTolerance(0.1);
    solver.setOptimizationStep(m_optimizationStepSpin->value());
    solver.setCurveSmoothness(m_curveSmoothnessSpin->value());
        
    // Set up callback for real-time statistics updates
    solver.setStatisticsCallback([this](const BezierDragChainSolver::SolverStatistics& stats) {
        onSolverStatisticsUpdated(stats);
        // Force GUI update to ensure real-time display
        QApplication::processEvents();
        
        // Show progress indicator during solving
        if (stats.iterationsUsed > 0) {
            m_segmentLengthsLabel->setText(QString("Segment Lengths: Min=%1, Max=%2, Avg=%3 mm (Solving...)")
                .arg(stats.minSegmentLength, 0, 'f', 1)
                .arg(stats.maxSegmentLength, 0, 'f', 1)
                .arg(stats.averageSegmentLength, 0, 'f', 1));
            m_solverIterationsLabel->setText(QString("Solver Iterations: %1/50 (Solving...)")
                .arg(stats.iterationsUsed));
        }
    });
    
    // Solve the drag chain path using strict angle constraints
    std::vector<BezierDragChainSegment> solvedSegments;
    
    // Prepare control points for the solver
    std::vector<QVector3D> controlPoints;
    std::vector<QVector3D> originalControlPoints; // Keep original 3D points for visualization
    
    // Use user-defined control points if available, otherwise auto-calculate
    if (!m_currentControlPoints.empty()) {
        // Use the existing control points, but project them to the 2D plane for solver
        qDebug() << "[ConnectionWidget] Using" << m_currentControlPoints.size() << "user-defined control points";
        for (const auto& controlPoint : m_currentControlPoints) {
            // Keep original 3D point for visualization
            originalControlPoints.push_back(controlPoint);
            
            // Project the control point to the 2D plane defined by start and end points for solver
            QVector3D projectedControlPoint = projectTo2DPlane(controlPoint, point1, point2);
            controlPoints.push_back(projectedControlPoint);
            qDebug() << "[ConnectionWidget] Using control point (original):" << controlPoint;
            qDebug() << "[ConnectionWidget] Using control point (projected):" << projectedControlPoint;
        }
    } else {
        // Auto-calculate a single control point in the 2D plane
        qDebug() << "[ConnectionWidget] Using auto-calculated control point";
        
        QVector3D midPoint = (point1 + point2) * 0.5f;
        QVector3D direction = (point2 - point1).normalized();
        
        // Calculate perpendicular axis in the 2D plane
        QVector3D perpendicular;
        if (std::abs(direction.x()) < 0.9f) {
            perpendicular = QVector3D::crossProduct(QVector3D(1, 0, 0), direction);
        } else {
            perpendicular = QVector3D::crossProduct(QVector3D(0, 1, 0), direction);
        }
        perpendicular.normalize();
        
        double distance = (point2 - point1).length();
        double controlDistance = distance * 0.3; // 30% of total distance
        
        QVector3D autoControlPoint = midPoint + perpendicular * controlDistance;
        controlPoints.push_back(autoControlPoint);
        originalControlPoints.push_back(autoControlPoint); // Same point for both
        qDebug() << "[ConnectionWidget] Auto-calculated control point:" << autoControlPoint;
    }
    
    // Use multi-control point solver if we have control points, otherwise use fallback
    if (!controlPoints.empty()) {
        qDebug() << "[ConnectionWidget] Using multi-control point solver with" << controlPoints.size() << "control points";
        for (size_t i = 0; i < controlPoints.size(); ++i) {
            qDebug() << "[ConnectionWidget]   Solver control point" << i << ":" << controlPoints[i];
        }
        qDebug() << "[ConnectionWidget] Calling solver.solveDragChainPathWithMultipleControlPoints...";
            solvedSegments = solver.solveDragChainPathWithMultipleControlPoints(
                point1, point2, controlPoints,
                pitchLength, bendRadius
            );
        qDebug() << "[ConnectionWidget] Solver returned" << solvedSegments.size() << "segments";
    } else {
        qDebug() << "[ConnectionWidget] No control points available, using fallback solver";
        // Fallback to single control point solver
        solvedSegments = solver.calculateSegmentsWithStrictAngleConstraints(
            point1, point2, QVector3D(),
            pitchLength, bendRadius,
            true  // Always enable path recovery
        );
    }
    
    qDebug() << "[ConnectionWidget] createDragChainPath: Solver returned" << solvedSegments.size() << "segments";
    if (!solvedSegments.empty()) {
        qDebug() << "[ConnectionWidget] First segment:" << solvedSegments[0].startPoint << "->" << solvedSegments[0].endPoint;
        qDebug() << "[ConnectionWidget] Last segment:" << solvedSegments.back().startPoint << "->" << solvedSegments.back().endPoint;
    }
    
    // Store solver segments for 2D visualization
    m_solverSegments = solvedSegments;
    qDebug() << "[ConnectionWidget] Stored" << m_solverSegments.size() << "solver segments for 2D visualization";
                
    // Store solver statistics for later use
    const auto& stats = solver.getLastSolverStatistics();
    qDebug() << "[ConnectionWidget] createDragChainPath: Retrieved solver statistics:";
    qDebug() << "[ConnectionWidget]   Raw stats - Distance to target:" << stats.finalDistanceToTarget << "mm";
    qDebug() << "[ConnectionWidget]   Raw stats - Converged:" << stats.converged;
    qDebug() << "[ConnectionWidget]   Raw stats - Iterations:" << stats.iterationsUsed;
    qDebug() << "[ConnectionWidget]   Raw stats - Computation time:" << stats.computationTime << "s";
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
    m_lastSolverStatistics.simulationTime = stats.computationTime;
    m_lastSolverStatistics.controlPointsOptimized = stats.controlPointsOptimized;
    m_lastSolverStatistics.jointsCreated = 0; // Not applicable for Bézier solver
    m_lastSolverStatistics.minSegmentLength = stats.minSegmentLength;
    m_lastSolverStatistics.maxSegmentLength = stats.maxSegmentLength;
    m_lastSolverStatistics.averageSegmentLength = stats.averageSegmentLength;
    m_lastSolverStatistics.minSegmentAngle = stats.minSegmentAngle;
    m_lastSolverStatistics.maxSegmentAngle = stats.maxSegmentAngle;
    m_lastSolverStatistics.averageSegmentAngle = stats.averageSegmentAngle;
    m_lastSolverStatistics.totalCableLength = stats.totalCableLength;
    m_lastSolverStatistics.totalSegments = stats.totalSegments;
    
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
    
    // No deviation analysis needed for straight-line segments
    // The segments follow the intended path exactly
    
    // Generate straight-line segments that follow the intended path (waypoints)
    // This matches what the user sees in the 2D visualization
    std::vector<CadOpenGLWidget::ConnectionPathSegment> segments;
    
    if (!solvedSegments.empty()) {
        qDebug() << "[ConnectionWidget] Generating straight-line segments from" << solvedSegments.size() << "solver segments";
        
        // Create waypoints from control points (start → control1 → control2 → ... → end)
        std::vector<QVector3D> waypoints;
        waypoints.push_back(point1);
        
        // Project all control points to the plane defined by start and end points
        for (const auto& controlPoint : originalControlPoints) {
            QVector3D projectedControlPoint = projectTo2DPlane(controlPoint, point1, point2);
            waypoints.push_back(projectedControlPoint);
            qDebug() << "[ConnectionWidget] Projected control point to plane:" << controlPoint << "->" << projectedControlPoint;
        }
        waypoints.push_back(point2);
        
        qDebug() << "[ConnectionWidget] Generated" << waypoints.size() << "waypoints for straight-line segments";
        for (size_t i = 0; i < waypoints.size(); ++i) {
            qDebug() << "[ConnectionWidget]   Waypoint" << i << ":" << waypoints[i];
        }
        
        // Generate straight-line segments between waypoints
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            QVector3D startPoint = waypoints[i];
            QVector3D endPoint = waypoints[i + 1];
            
            // Calculate distance between waypoints
            double distance = (endPoint - startPoint).length();
            int segmentCount = static_cast<int>(std::ceil(distance / pitchLength));
            
            if (segmentCount < 1) segmentCount = 1;
            
            qDebug() << "[ConnectionWidget] Waypoint segment" << i << ":" << startPoint << "->" << endPoint;
            qDebug() << "[ConnectionWidget]   Distance:" << distance << "mm, generating" << segmentCount << "segments";
            
            // Generate segments along the straight line
            for (int j = 0; j < segmentCount; ++j) {
                double t = static_cast<double>(j) / segmentCount;
                QVector3D segmentStart = startPoint + t * (endPoint - startPoint);
                
                double nextT = static_cast<double>(j + 1) / segmentCount;
                if (j == segmentCount - 1) nextT = 1.0;
                QVector3D segmentEnd = startPoint + nextT * (endPoint - startPoint);
                
                // Use alternating colors for better visualization
            QVector4D segmentColor;
                if (i % 2 == 0) {
                segmentColor = QVector4D(0.0f, 1.0f, 0.0f, 0.8f); // Green
            } else {
                segmentColor = QVector4D(0.0f, 0.0f, 1.0f, 0.8f); // Blue
            }
            
                // Create OpenGL segment
            segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
                    segmentStart, segmentEnd, segmentColor, 
                m_chainWidthSpin->value(), m_chainHeightSpin->value(), false));
            
                qDebug() << "[ConnectionWidget] Created straight-line segment" << segments.size() - 1 << ":" << segmentStart << "->" << segmentEnd;
            }
        }
        
        qDebug() << "[ConnectionWidget] Generated" << segments.size() << "straight-line segments";
        
        qDebug() << "[ConnectionWidget] Created" << segments.size() << "straight-line OpenGL segments";
        
        // Update GUI to show compatibility information
        m_compatibilityLabel->setText("Compatibility: ✓ Straight-line segments generated");
            m_compatibilityLabel->setStyleSheet("color: green; font-weight: bold;");
    } else {
        qDebug() << "[ConnectionWidget] WARNING: No segments generated by solver!";
        
        // Fallback: create a simple straight line segment
        qDebug() << "[ConnectionWidget] Creating fallback straight line segment";
        segments.push_back(CadOpenGLWidget::ConnectionPathSegment(
            point1, point2, QVector4D(1.0f, 0.0f, 0.0f, 0.8f), 
            m_chainWidthSpin->value(), m_chainHeightSpin->value(), false));
    }
    
    // Store the control points for visualization (including auto-calculated ones)
    m_currentControlPoints.clear();
    for (const auto& controlPoint : originalControlPoints) {
        m_currentControlPoints.push_back(controlPoint);
    }
    qDebug() << "[ConnectionWidget] Stored" << m_currentControlPoints.size() << "original 3D control points for visualization";
    
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
        
        QString iterationsText = QString("Solver: Mathematical calculation (%.3f s) [%1 segments, pitch: %.1f mm]")
            .arg(m_lastSolverStatistics.simulationTime, 0, 'f', 3)
            .arg(m_lastSolverStatistics.controlPointsOptimized)
            .arg(m_pitchLengthSpin->value());
        m_solverIterationsLabel->setText(iterationsText);
        
        // Display cable length and segment count
        QString cableLengthText = QString("Cable Length: %1 mm")
            .arg(m_lastSolverStatistics.totalCableLength, 0, 'f', 1);
        m_cableLengthLabel->setText(cableLengthText);
        
        QString segmentCountText = QString("Segment Count: %1")
            .arg(m_lastSolverStatistics.totalSegments);
        m_segmentCountLabel->setText(segmentCountText);
        
        // Display segment angle statistics
        QString segmentAnglesText = QString("Segment Angles: Min=%1°, Max=%2°, Avg=%3°")
            .arg(m_lastSolverStatistics.minSegmentAngle * 180.0 / M_PI, 0, 'f', 1)
            .arg(m_lastSolverStatistics.maxSegmentAngle * 180.0 / M_PI, 0, 'f', 1)
            .arg(m_lastSolverStatistics.averageSegmentAngle * 180.0 / M_PI, 0, 'f', 1);
        m_segmentAnglesLabel->setText(segmentAnglesText);
        
        // Check for violations and update compatibility label
        double maxAllowedAngle = 2.0 * std::asin(m_pitchLengthSpin->value() / (2.0 * m_maxBendRadiusSpin->value()));
        bool hasViolations = (m_lastSolverStatistics.maxSegmentAngle > 0.0 && 
                             m_lastSolverStatistics.maxSegmentAngle > maxAllowedAngle);
        
        if (hasViolations) {
            m_compatibilityLabel->setText("Compatibility: ⚠️ Some segments exceed bend radius");
            m_compatibilityLabel->setStyleSheet("color: orange; font-weight: bold;");
        } else {
            m_compatibilityLabel->setText("Compatibility: ✓ All segments within constraints");
            m_compatibilityLabel->setStyleSheet("color: green; font-weight: bold;");
        }
        
        qDebug() << "[ConnectionWidget] Updated GUI labels from stored statistics:";
        qDebug() << "[ConnectionWidget]   Segment lengths label:" << segmentLengthsText;
        qDebug() << "[ConnectionWidget]   Iterations label:" << iterationsText;
        
        // Force GUI update to ensure labels are displayed
        QApplication::processEvents();
    } else {
        qDebug() << "[ConnectionWidget] No stored solver statistics available";
        m_segmentLengthsLabel->setText("Segment Lengths: --");
        m_solverIterationsLabel->setText("Solver Iterations: --");
        m_cableLengthLabel->setText("Cable Length: --");
        m_segmentCountLabel->setText("Segment Count: --");
        m_segmentAnglesLabel->setText("Segment Angles: --");
        
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
        m_cableLengthLabel->setText("Cable Length: --");
        m_segmentCountLabel->setText("Segment Count: --");
        m_segmentAnglesLabel->setText("Segment Angles: --");
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
            totalSegments = m_lastSolverStatistics.controlPointsOptimized;
            baseSegments = m_lastSolverStatistics.controlPointsOptimized;
        } else {
            qDebug() << "[ConnectionWidget] updateConnectionInfo: No solver statistics available yet";
            // Don't call the solver here - let updateConnectionPathVisualization handle it
            
            // Use fallback calculation
            effectiveLength = distance;
            totalSegments = static_cast<int>(std::ceil(distance / m_pitchLengthSpin->value()));
            baseSegments = static_cast<int>(std::ceil(distance / m_pitchLengthSpin->value()));
        }
            } else {
            // Use standard calculation with pitch length and segments
            double pitchLength = m_pitchLengthSpin->value();
            int segmentCount = static_cast<int>(std::ceil(distance / pitchLength));
            
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
            connectionData->segmentCount = m_lastSolverStatistics.controlPointsOptimized;
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

void ConnectionCreationWidget::onSolverStatisticsUpdated(const BezierDragChainSolver::SolverStatistics& stats)
{
    qDebug() << "[ConnectionWidget] onSolverStatisticsUpdated: Real-time statistics update received";
    qDebug() << "[ConnectionWidget]   Min segment length:" << stats.minSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Max segment length:" << stats.maxSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Avg segment length:" << stats.averageSegmentLength << "mm";
    qDebug() << "[ConnectionWidget]   Converged:" << stats.converged;
    qDebug() << "[ConnectionWidget]   Iterations:" << stats.iterationsUsed;
    qDebug() << "[ConnectionWidget]   Computation time:" << stats.computationTime << "s";
    
    // Update stored statistics
    m_lastSolverStatistics.iterationsUsed = stats.iterationsUsed;
    m_lastSolverStatistics.finalDistanceToTarget = stats.finalDistanceToTarget;
    m_lastSolverStatistics.converged = stats.converged;
    m_lastSolverStatistics.simulationTime = stats.computationTime;
    m_lastSolverStatistics.controlPointsOptimized = stats.controlPointsOptimized;
    m_lastSolverStatistics.jointsCreated = 0; // Not applicable for Bézier solver
    m_lastSolverStatistics.minSegmentLength = stats.minSegmentLength;
    m_lastSolverStatistics.maxSegmentLength = stats.maxSegmentLength;
    m_lastSolverStatistics.averageSegmentLength = stats.averageSegmentLength;
    m_lastSolverStatistics.minSegmentAngle = stats.minSegmentAngle;
    m_lastSolverStatistics.maxSegmentAngle = stats.maxSegmentAngle;
    m_lastSolverStatistics.averageSegmentAngle = stats.averageSegmentAngle;
    m_lastSolverStatistics.totalCableLength = stats.totalCableLength;
    m_lastSolverStatistics.totalSegments = stats.totalSegments;
    
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
    
    // Reset Bézier curve solver parameters
    m_optimizationStepSpin->setValue(0.1);
    m_curveSmoothnessSpin->setValue(1.0);
    
    // Enhanced visualization, strict angle constraints, and path recovery are always enabled
    
    // Reset control point settings
    m_currentControlPoints.clear();
    m_solverSegments.clear();
    m_userModifiedControlPoint = false;
    
    // Clear solver statistics
    clearSolverStatistics();
    
    // Clear visualizations
    if (m_openGLWidget) {
        m_openGLWidget->clearConnectionPathSegments();
        m_openGLWidget->clearControlPointMarkers();
    }
    
    if (m_2dVisualization) {
        m_2dVisualization->clear();
    }
    
    // Update visualization
    updateConnectionPathVisualization();
    updateSolverStatisticsDisplay();
    
    qDebug() << "[ConnectionWidget] Widget state cleared";
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
    qDebug() << "[ConnectionWidget] Current control points:" << m_currentControlPoints.size();
    for (size_t i = 0; i < m_currentControlPoints.size(); ++i) {
        qDebug() << "[ConnectionWidget]   Control point" << i << ":" << m_currentControlPoints[i];
    }
    
    if (!m_openGLWidget) {
        qDebug() << "[ConnectionWidget] No OpenGL widget available";
        return;
    }
    
    if (!m_point1 || !m_point2) {
        qDebug() << "[ConnectionWidget] One or both points not selected, clearing segments";
        m_openGLWidget->clearConnectionPathSegments();
        m_openGLWidget->clearControlPointMarkers();
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
    bool isFlexible = m_isFlexibleCheck->isChecked();
    bool isDragChain = (m_connectionTypeCombo->currentData().toInt() == static_cast<int>(ConnectionNodeData::ConnectionType::DragChain));
    
    qDebug() << "[ConnectionWidget] Connection parameters:";
    qDebug() << "[ConnectionWidget]   isFlexible:" << isFlexible;
    qDebug() << "[ConnectionWidget]   isDragChain:" << isDragChain;
    qDebug() << "[ConnectionWidget]   bendRadius:" << bendRadius;
    qDebug() << "[ConnectionWidget]   pitchLength:" << pitchLength;
    qDebug() << "[ConnectionWidget]   connectionTypeCombo currentData:" << m_connectionTypeCombo->currentData().toInt();
    
    qDebug() << "[ConnectionWidget] Connection parameters - bendRadius:" << bendRadius << "pitchLength:" << pitchLength << "isFlexible:" << isFlexible;
    qDebug() << "[ConnectionWidget] Drag chain settings - isDragChain:" << isDragChain;
    
    if (isFlexible && bendRadius > 0.0) {
        if (isDragChain) {
            qDebug() << "[ConnectionWidget] Creating unified drag chain path";
            qDebug() << "[ConnectionWidget]   Control points available:" << m_currentControlPoints.size();
            for (size_t i = 0; i < m_currentControlPoints.size(); ++i) {
                qDebug() << "[ConnectionWidget]     Control point" << i << ":" << m_currentControlPoints[i];
            }
            
            // Always use solver for drag chains to enforce constraints properly
            qDebug() << "[ConnectionWidget]   Will call solver (drag chain requires constraint enforcement)";
            segments = createDragChainPath(point1, point2, direction, distance, bendRadius, pitchLength);
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
    for (size_t i = 0; i < segments.size(); ++i) {
        qDebug() << "[ConnectionWidget]   Passing 3D segment" << i << ":" << segments[i].start << "->" << segments[i].end 
                 << "with color" << segments[i].color << "and dimensions" << segments[i].width << "x" << segments[i].height;
    }
    
    // Check if segments have reasonable dimensions
    bool hasVisibleSegments = false;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (segments[i].width >= 1.0 && segments[i].height >= 1.0) {
            hasVisibleSegments = true;
            break;
        }
    }
    
    if (!hasVisibleSegments) {
        qDebug() << "[ConnectionWidget] WARNING: All segments have very small dimensions! This might be why they're not visible.";
    }
    
    m_openGLWidget->setConnectionPathSegments(segments);
    qDebug() << "[ConnectionWidget] setConnectionPathSegments call completed";
    
    // Create and set control point markers
    std::vector<CadOpenGLWidget::ControlPointMarker> controlPointMarkers;
    
    // Use stored control points (including auto-calculated ones) projected to the plane
    for (size_t i = 0; i < m_currentControlPoints.size(); ++i) {
        const auto& controlPoint = m_currentControlPoints[i];
        // Project the control point to the plane defined by start and end points
        QVector3D projectedControlPoint = projectTo2DPlane(controlPoint, point1, point2);
        QString label = QString("CP%1").arg(i + 1);
        
        // Use different colors for different control points (matching 2D visualization)
        QVector4D color;
        switch (i % 4) {
            case 0: color = QVector4D(0.0f, 0.0f, 1.0f, 0.8f); break; // Blue
            case 1: color = QVector4D(1.0f, 0.0f, 1.0f, 0.8f); break; // Magenta
            case 2: color = QVector4D(0.0f, 1.0f, 1.0f, 0.8f); break; // Cyan
            case 3: color = QVector4D(1.0f, 1.0f, 0.0f, 0.8f); break; // Yellow
        }
        
        controlPointMarkers.emplace_back(projectedControlPoint, color, 12.0f, label);
        qDebug() << "[ConnectionWidget] Added control point marker" << i << "at" << projectedControlPoint << "with label" << label;
    }
    
    // Set the control point markers in the OpenGL widget
    m_openGLWidget->setControlPointMarkers(controlPointMarkers);
    qDebug() << "[ConnectionWidget] setControlPointMarkers call completed with" << controlPointMarkers.size() << "markers";
    
    // Force OpenGL widget to redraw with new segments and markers
    if (m_openGLWidget) {
        qDebug() << "[ConnectionWidget] About to force OpenGL widget redraw with" << segments.size() << "segments";
        m_openGLWidget->update();
        m_openGLWidget->repaint(); // Force immediate redraw
        qDebug() << "[ConnectionWidget] Forced OpenGL widget redraw";
    } else {
        qDebug() << "[ConnectionWidget] WARNING: OpenGL widget is null!";
    }
    
    // Update 2D visualization
    if (m_2dVisualization) {
        m_2dVisualization->setStartPoint(point1);
        m_2dVisualization->setEndPoint(point2);
        
        // Only update control points that haven't been manually modified by the user
        // This prevents the solver from overriding user's control point positions
        std::vector<QVector3D> controlPointsToUpdate = m_currentControlPoints;
        bool hasUserModifiedPoints = !m_userModifiedControlPointIndices.empty();
        
        if (hasUserModifiedPoints) {
            qDebug() << "[ConnectionWidget] User has modified control points, preserving user positions";
            // Don't update the 2D visualization with solver results
            // The 2D visualization already has the user's positions
        } else {
            m_2dVisualization->setControlPoints(controlPointsToUpdate);
            qDebug() << "[ConnectionWidget] Updated 2D visualization with solver control points";
        }
        
        m_2dVisualization->setPitchLength(pitchLength);
        m_2dVisualization->setBendRadius(bendRadius);
        
        // Pass solver segments to 2D visualization for green/red point display
        if (isDragChain && !m_solverSegments.empty()) {
            m_2dVisualization->setSolverSegments(m_solverSegments);
            qDebug() << "[ConnectionWidget] Passed" << m_solverSegments.size() << "solver segments to 2D visualization";
        } else {
            qDebug() << "[ConnectionWidget] No solver segments available for 2D visualization";
        }
    }
    
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



void ConnectionCreationWidget::onStartPointChanged(const QVector3D& point)
{
    qDebug() << "[ConnectionWidget] 2D visualization changed start point to:" << point;
    // Update the 2D visualization with new start point
    m_2dVisualization->setStartPoint(point);
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onEndPointChanged(const QVector3D& point)
{
    qDebug() << "[ConnectionWidget] 2D visualization changed end point to:" << point;
    // Update the 2D visualization with new end point
    m_2dVisualization->setEndPoint(point);
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onControlPointChanged(int index, const QVector3D& point)
{
    qDebug() << "[ConnectionWidget] 2D visualization changed control point" << index << "to:" << point;
    qDebug() << "[ConnectionWidget] Current control points before update:" << m_currentControlPoints.size();
    for (size_t i = 0; i < m_currentControlPoints.size(); ++i) {
        qDebug() << "[ConnectionWidget]   Control point" << i << ":" << m_currentControlPoints[i];
    }
    
    qDebug() << "[ConnectionWidget] Solver segments before update:" << m_solverSegments.size();
    
    // User modified a control point
    m_userModifiedControlPoint = true;
    m_userModifiedControlPointIndices.insert(index);
    
    // Use the exact point that the user moved (no projection here)
    // The projection will be done in createDragChainPath when needed
    QVector3D userPoint = point;
    
    // Update the current control points with the user's exact position
    if (index >= static_cast<int>(m_currentControlPoints.size())) {
        m_currentControlPoints.resize(index + 1);
    }
    m_currentControlPoints[index] = userPoint;
    
    qDebug() << "[ConnectionWidget] Updated control point" << index << "to:" << userPoint;
    qDebug() << "[ConnectionWidget] Current control points after update:" << m_currentControlPoints.size();
    for (size_t i = 0; i < m_currentControlPoints.size(); ++i) {
        qDebug() << "[ConnectionWidget]   Control point" << i << ":" << m_currentControlPoints[i];
    }
    
    // Update 2D visualization with the updated control points
    if (m_2dVisualization) {
        qDebug() << "[ConnectionWidget] Updating 2D visualization with" << m_currentControlPoints.size() << "control points";
        m_2dVisualization->setControlPoints(m_currentControlPoints);
        m_2dVisualization->update(); // Force immediate redraw
    }
    
    // Clear solver statistics to force re-calculation with new control point
    clearSolverStatistics();
    
    // Reset the flag since we're about to re-run the solver
    m_userModifiedControlPoint = false;
    
    // Update visualization to re-run solver with new control point
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onDeviationThresholdChanged(double value)
{
    qDebug() << "[ConnectionWidget] Deviation threshold changed to:" << value << "mm";
    
    // Update 2D visualization with new threshold
    if (m_2dVisualization) {
        m_2dVisualization->setDeviationThreshold(value);
    }
    
    // Re-run path creation with new threshold
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onChainDimensionsChanged(double value)
{
    qDebug() << "[ConnectionWidget] Chain dimensions changed - width:" << m_chainWidthSpin->value() 
             << "mm, height:" << m_chainHeightSpin->value() << "mm";
    
    // Re-run path creation with new dimensions
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onControlPointAdded(int index, const QVector3D& point)
{
    qDebug() << "[ConnectionWidget] Control point added at index" << index << "position:" << point;
    
    // User modified control points
    m_userModifiedControlPoint = true;
    
    // Update the current control points vector
    if (index >= static_cast<int>(m_currentControlPoints.size())) {
        m_currentControlPoints.resize(index + 1);
    }
    m_currentControlPoints[index] = point;
    
    // Update 2D visualization with the updated control points
    if (m_2dVisualization) {
        qDebug() << "[ConnectionWidget] Updating 2D visualization with" << m_currentControlPoints.size() << "control points";
        m_2dVisualization->setControlPoints(m_currentControlPoints);
        m_2dVisualization->update(); // Force immediate redraw
    }
    
    // Clear solver statistics to force re-calculation with new control point
    clearSolverStatistics();
    
    // Reset the flag since we're about to re-run the solver
    m_userModifiedControlPoint = false;
    
    // Update visualization to re-run solver with new control point
    updateConnectionPathVisualization();
}

void ConnectionCreationWidget::onControlPointRemoved(int index)
{
    qDebug() << "[ConnectionWidget] Control point removed at index" << index;
    
    // User modified control points
    m_userModifiedControlPoint = true;
    
    // Remove the control point from the vector
    if (index >= 0 && index < static_cast<int>(m_currentControlPoints.size())) {
        m_currentControlPoints.erase(m_currentControlPoints.begin() + index);
    }
    
    // Update 2D visualization with the updated control points
    if (m_2dVisualization) {
        qDebug() << "[ConnectionWidget] Updating 2D visualization with" << m_currentControlPoints.size() << "control points";
        m_2dVisualization->setControlPoints(m_currentControlPoints);
        m_2dVisualization->update(); // Force immediate redraw
    }
    
    // Clear solver statistics to force re-calculation with updated control points
    clearSolverStatistics();
    
    // Reset the flag since we're about to re-run the solver
    m_userModifiedControlPoint = false;
    
    // Update visualization to re-run solver with updated control points
    updateConnectionPathVisualization();
}

// 2D Visualization Widget Implementation
DragChain2DVisualization::DragChain2DVisualization(QWidget* parent)
    : QFrame(parent)
    , m_pitchLength(18.0)
    , m_bendRadius(18.0)
    , m_hasStartPoint(false)
    , m_hasEndPoint(false)
    , m_selectedPointIndex(-1)
    , m_dragging(false)
    , m_deviationThreshold(1.0) // 1mm threshold for deviation detection
{
    setFrameStyle(QFrame::Box);
    setLineWidth(1);
    setMidLineWidth(0);
    setStyleSheet("QFrame { background-color: white; border: 1px solid gray; }");
    
    // Enable mouse tracking for hover effects
    setMouseTracking(true);
    
    // Set focus policy for keyboard events
    setFocusPolicy(Qt::StrongFocus);
}

void DragChain2DVisualization::setStartPoint(const QVector3D& point)
{
    m_startPoint = point;
    m_hasStartPoint = true;
    updateTransform();
    update();
}

void DragChain2DVisualization::setEndPoint(const QVector3D& point)
{
    m_endPoint = point;
    m_hasEndPoint = true;
    updateTransform();
    update();
}

void DragChain2DVisualization::setControlPoints(const std::vector<QVector3D>& points)
{
    qDebug() << "[2DVisualization] Setting control points:" << points.size() << "points";
    for (size_t i = 0; i < points.size(); ++i) {
        qDebug() << "[2DVisualization]   Control point" << i << ":" << points[i];
    }
    
    m_controlPoints = points;
    updateTransform();
    update();
}

void DragChain2DVisualization::setPitchLength(double pitchLength)
{
    m_pitchLength = pitchLength;
    update();
}

void DragChain2DVisualization::setBendRadius(double bendRadius)
{
    m_bendRadius = bendRadius;
    update();
}

void DragChain2DVisualization::clear()
{
    m_hasStartPoint = false;
    m_hasEndPoint = false;
    m_controlPoints.clear();
    m_selectedPointIndex = -1;
    m_dragging = false;
    update();
}

void DragChain2DVisualization::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // Draw background
    painter.fillRect(rect(), Qt::white);
    
    // Update transform if needed
    updateTransform();
    
    // Draw grid
    drawGrid(painter);
    
    // Draw Bézier curve
    drawBezierCurve(painter);
    
    // Draw segments
    drawSegments(painter);
    
    // Draw points
    drawPoints(painter);
}

void DragChain2DVisualization::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        QPoint pos = event->pos();
        QVector3D worldPos = screenToWorld(pos);
        
        // Check if clicking on a control point
        for (size_t i = 0; i < m_controlPoints.size(); ++i) {
            QPoint screenPoint = worldToScreen(m_controlPoints[i]);
            if ((pos - screenPoint).manhattanLength() < 10) {
                m_selectedPointIndex = static_cast<int>(i);
                m_dragging = true;
                m_lastMousePos = pos;
                update();
                return;
            }
        }
        
        // Check if clicking on start or end point
        if (m_hasStartPoint) {
            QPoint screenPoint = worldToScreen(m_startPoint);
            if ((pos - screenPoint).manhattanLength() < 10) {
                m_selectedPointIndex = -2; // Start point
                m_dragging = true;
                m_lastMousePos = pos;
                update();
                return;
            }
        }
        
        if (m_hasEndPoint) {
            QPoint screenPoint = worldToScreen(m_endPoint);
            if ((pos - screenPoint).manhattanLength() < 10) {
                m_selectedPointIndex = -3; // End point
                m_dragging = true;
                m_lastMousePos = pos;
                update();
                return;
            }
        }
        
        // If not clicking on any point, deselect
        m_selectedPointIndex = -1;
        m_dragging = false;
        update();
    }
}

void DragChain2DVisualization::mouseMoveEvent(QMouseEvent* event)
{
    if (m_dragging && m_selectedPointIndex >= -3) {
        QPoint pos = event->pos();
        QVector3D worldPos = screenToWorld(pos);
        
        if (m_selectedPointIndex == -2) {
            // Moving start point
            m_startPoint = worldPos;
            emit startPointChanged(worldPos);
        } else if (m_selectedPointIndex == -3) {
            // Moving end point
            m_endPoint = worldPos;
            emit endPointChanged(worldPos);
        } else if (m_selectedPointIndex >= 0) {
            // Moving control point
            qDebug() << "[2DVisualization] Moving control point" << m_selectedPointIndex << "to" << worldPos;
            m_controlPoints[m_selectedPointIndex] = worldPos;
            emit controlPointChanged(m_selectedPointIndex, worldPos);
            
            // Force immediate update to ensure the main widget gets the change
            update();
        }
        
        update();
    }
}

void DragChain2DVisualization::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        m_dragging = false;
    }
}

void DragChain2DVisualization::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        QPoint pos = event->pos();
        QVector3D worldPos = screenToWorld(pos);
        
        // Check if double-clicking on an existing point (don't add duplicate)
        for (size_t i = 0; i < m_controlPoints.size(); ++i) {
            QPoint screenPoint = worldToScreen(m_controlPoints[i]);
            if ((pos - screenPoint).manhattanLength() < 10) {
                return; // Don't add if clicking on existing point
            }
        }
        
        // Check if double-clicking on start or end points (don't add)
        if (m_hasStartPoint) {
            QPoint screenPoint = worldToScreen(m_startPoint);
            if ((pos - screenPoint).manhattanLength() < 10) {
                return;
            }
        }
        
        if (m_hasEndPoint) {
            QPoint screenPoint = worldToScreen(m_endPoint);
            if ((pos - screenPoint).manhattanLength() < 10) {
                return;
            }
        }
        
        // Add new control point at the clicked location
        int newIndex = static_cast<int>(m_controlPoints.size());
        m_controlPoints.push_back(worldPos);
        m_selectedPointIndex = newIndex;
        
        emit controlPointAdded(newIndex, worldPos);
        update();
    }
}

void DragChain2DVisualization::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
        if (m_selectedPointIndex >= 0 && m_selectedPointIndex < static_cast<int>(m_controlPoints.size())) {
            int indexToRemove = m_selectedPointIndex;
            m_controlPoints.erase(m_controlPoints.begin() + indexToRemove);
            
            // Adjust selected point index if needed
            if (m_selectedPointIndex >= static_cast<int>(m_controlPoints.size())) {
                m_selectedPointIndex = static_cast<int>(m_controlPoints.size()) - 1;
            }
            
            emit controlPointRemoved(indexToRemove);
            update();
        }
    }
}

QPoint DragChain2DVisualization::worldToScreen(const QVector3D& worldPos) const
{
    QPointF transformed = m_transform.map(QPointF(worldPos.x(), worldPos.y()));
    return transformed.toPoint();
}

QVector3D DragChain2DVisualization::screenToWorld(const QPoint& screenPos) const
{
    QPointF transformed = m_transform.inverted().map(QPointF(screenPos));
    return QVector3D(transformed.x(), transformed.y(), 0.0f);
}

void DragChain2DVisualization::updateTransform()
{
    // Calculate world bounds
    QVector3D minPoint, maxPoint;
    bool firstPoint = true;
    
    if (m_hasStartPoint) {
        minPoint = maxPoint = m_startPoint;
        firstPoint = false;
    }
    
    if (m_hasEndPoint) {
        if (firstPoint) {
            minPoint = maxPoint = m_endPoint;
            firstPoint = false;
        } else {
            minPoint.setX(std::min(minPoint.x(), m_endPoint.x()));
            minPoint.setY(std::min(minPoint.y(), m_endPoint.y()));
            maxPoint.setX(std::max(maxPoint.x(), m_endPoint.x()));
            maxPoint.setY(std::max(maxPoint.y(), m_endPoint.y()));
        }
    }
    
    for (const auto& point : m_controlPoints) {
        if (firstPoint) {
            minPoint = maxPoint = point;
            firstPoint = false;
        } else {
            minPoint.setX(std::min(minPoint.x(), point.x()));
            minPoint.setY(std::min(minPoint.y(), point.y()));
            maxPoint.setX(std::max(maxPoint.x(), point.x()));
            maxPoint.setY(std::max(maxPoint.y(), point.y()));
        }
    }
    
    // Add padding
    double padding = 50.0;
    m_worldBounds = QRectF(minPoint.x() - padding, minPoint.y() - padding,
                           maxPoint.x() - minPoint.x() + 2 * padding,
                           maxPoint.y() - minPoint.y() + 2 * padding);
    
    // If no points, use default bounds
    if (firstPoint) {
        m_worldBounds = QRectF(-100, -100, 200, 200);
    }
    
    m_screenBounds = rect();
    
    // Create transform from world to screen
    double scaleX = m_screenBounds.width() / m_worldBounds.width();
    double scaleY = m_screenBounds.height() / m_worldBounds.height();
    double scale = std::min(scaleX, scaleY);
    
    m_transform.reset();
    m_transform.translate(m_screenBounds.center().x(), m_screenBounds.center().y());
    m_transform.scale(scale, scale);
    m_transform.translate(-m_worldBounds.center().x(), -m_worldBounds.center().y());
}

void DragChain2DVisualization::drawGrid(QPainter& painter)
{
    painter.setPen(QPen(Qt::lightGray, 1, Qt::DotLine));
    
    // Draw grid lines every 10 units
    double gridSpacing = 10.0;
    QRectF worldRect = m_worldBounds;
    
    // Vertical lines
    for (double x = worldRect.left(); x <= worldRect.right(); x += gridSpacing) {
        QPoint p1 = worldToScreen(QVector3D(x, worldRect.top(), 0));
        QPoint p2 = worldToScreen(QVector3D(x, worldRect.bottom(), 0));
        painter.drawLine(p1, p2);
    }
    
    // Horizontal lines
    for (double y = worldRect.top(); y <= worldRect.bottom(); y += gridSpacing) {
        QPoint p1 = worldToScreen(QVector3D(worldRect.left(), y, 0));
        QPoint p2 = worldToScreen(QVector3D(worldRect.right(), y, 0));
        painter.drawLine(p1, p2);
    }
}

void DragChain2DVisualization::drawPoints(QPainter& painter)
{
    // Draw start point
    if (m_hasStartPoint) {
        QPoint screenPos = worldToScreen(m_startPoint);
        painter.setPen(QPen(Qt::green, 2));
        painter.setBrush(QBrush(Qt::green));
        painter.drawEllipse(screenPos, 6, 6);
        
        // Draw label
        painter.setPen(Qt::black);
        painter.drawText(screenPos + QPoint(10, -10), "Start");
    }
    
    // Draw end point
    if (m_hasEndPoint) {
        QPoint screenPos = worldToScreen(m_endPoint);
        painter.setPen(QPen(Qt::red, 2));
        painter.setBrush(QBrush(Qt::red));
        painter.drawEllipse(screenPos, 6, 6);
        
        // Draw label
        painter.setPen(Qt::black);
        painter.drawText(screenPos + QPoint(10, -10), "End");
    }
    
    // Draw control points
    for (size_t i = 0; i < m_controlPoints.size(); ++i) {
        QPoint screenPos = worldToScreen(m_controlPoints[i]);
        
        // Use different colors for different control points
        QColor color;
        switch (i % 4) {
            case 0: color = Qt::blue; break;
            case 1: color = Qt::magenta; break;
            case 2: color = Qt::cyan; break;
            case 3: color = Qt::darkYellow; break;
        }
        
        painter.setPen(QPen(color, 2));
        painter.setBrush(QBrush(color));
        painter.drawEllipse(screenPos, 5, 5);
        
        // Draw label
        painter.setPen(Qt::black);
        painter.drawText(screenPos + QPoint(10, -10), QString("CP%1").arg(i + 1));
    }
    
    // Highlight selected point
    if (m_selectedPointIndex >= -3) {
        QPoint screenPos;
        if (m_selectedPointIndex == -2) {
            screenPos = worldToScreen(m_startPoint);
        } else if (m_selectedPointIndex == -3) {
            screenPos = worldToScreen(m_endPoint);
        } else if (m_selectedPointIndex >= 0) {
            screenPos = worldToScreen(m_controlPoints[m_selectedPointIndex]);
        }
        
        painter.setPen(QPen(Qt::black, 3));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(screenPos, 10, 10);
    }
}

void DragChain2DVisualization::drawBezierCurve(QPainter& painter)
{
    if (!m_hasStartPoint || !m_hasEndPoint) return;
    
    // Create a path through all waypoints (start → control1 → control2 → ... → end)
    std::vector<QVector3D> waypoints;
    waypoints.push_back(m_startPoint);
    waypoints.insert(waypoints.end(), m_controlPoints.begin(), m_controlPoints.end());
    waypoints.push_back(m_endPoint);
    
    if (waypoints.size() < 2) return;
    
    qDebug() << "[2DVisualization] Drawing blue curve with" << waypoints.size() << "waypoints";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        qDebug() << "[2DVisualization]   Waypoint" << i << ":" << waypoints[i];
    }
    
    painter.setPen(QPen(Qt::blue, 2, Qt::SolidLine));
    
    // Draw straight lines between consecutive waypoints (this matches what the solver does)
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        QPoint startScreen = worldToScreen(waypoints[i]);
        QPoint endScreen = worldToScreen(waypoints[i + 1]);
        painter.drawLine(startScreen, endScreen);
        qDebug() << "[2DVisualization]   Drawing line from" << startScreen << "to" << endScreen;
    }
}

void DragChain2DVisualization::drawSegments(QPainter& painter)
{
    if (!m_hasStartPoint || !m_hasEndPoint) return;
    
    // Create a series of Bézier curves through control points
    std::vector<QVector3D> curvePoints;
    curvePoints.push_back(m_startPoint);
    curvePoints.insert(curvePoints.end(), m_controlPoints.begin(), m_controlPoints.end());
    curvePoints.push_back(m_endPoint);
    
    if (curvePoints.size() < 3) return;
    
    // Draw both the ideal Bézier curve (dashed) and the actual solver path (solid)
    
    // First, draw the ideal path segments (dashed line) - straight lines between waypoints
    painter.setPen(QPen(Qt::lightGray, 2, Qt::DashLine));
    
    for (size_t i = 0; i < curvePoints.size() - 1; ++i) {
        QVector3D startPoint = curvePoints[i];
        QVector3D endPoint = curvePoints[i + 1];
        
        // Calculate distance between waypoints
        double distance = (endPoint - startPoint).length();
        int segmentCount = static_cast<int>(std::ceil(distance / m_pitchLength));
        
        if (segmentCount < 1) continue;
        
        // Generate segment points along the straight line
        for (int j = 0; j < segmentCount; ++j) {
            double t = static_cast<double>(j) / segmentCount;
            QVector3D segmentStart = startPoint + t * (endPoint - startPoint);
            
            double nextT = static_cast<double>(j + 1) / segmentCount;
            if (j == segmentCount - 1) nextT = 1.0;
            QVector3D segmentEnd = startPoint + nextT * (endPoint - startPoint);
            
            // Draw the ideal segment (dashed)
            QPoint startScreen = worldToScreen(segmentStart);
            QPoint endScreen = worldToScreen(segmentEnd);
            painter.drawLine(startScreen, endScreen);
        }
    }
    
    // Now draw the actual solver-generated path (solid line) with deviation highlighting
    if (!m_solverSegments.empty()) {
        qDebug() << "[2DVisualization] Drawing" << m_solverSegments.size() << "actual solver segments";
        
        for (size_t i = 0; i < m_solverSegments.size(); ++i) {
            const auto& segment = m_solverSegments[i];
            
            QPoint startScreen = worldToScreen(segment.startPoint);
            QPoint endScreen = worldToScreen(segment.endPoint);
            
            // Check if this segment causes deviation
            bool isDeviationSegment = false;
            double distanceToTarget = 0.0;
            
            if (i < m_segmentDeviations.size()) {
                isDeviationSegment = m_segmentDeviations[i];
            }
            
            if (i < m_segmentDistancesToTarget.size()) {
                distanceToTarget = m_segmentDistancesToTarget[i];
            }
            
            // Check for bend angle violations
            bool violatesBendAngle = false;
            if (segment.isBend && segment.bendRadius > 0.0) {
                // Calculate maximum allowed bend angle based on bend radius constraint
                double maxBendAngle = 2.0 * std::asin(m_pitchLength / (2.0 * m_bendRadius));
                violatesBendAngle = segment.bendAngle > maxBendAngle;
            }
            
            // Set color based on violation type (bend angle violations take priority)
            if (violatesBendAngle) {
                // Orange for bend angle violations
                painter.setPen(QPen(Qt::darkYellow, 4, Qt::SolidLine));
            } else if (isDeviationSegment) {
                // Red for segments that cause deviation
                painter.setPen(QPen(Qt::red, 4, Qt::SolidLine));
            } else {
                // Green for normal segments
    painter.setPen(QPen(Qt::darkGreen, 3, Qt::SolidLine));
            }
            
            painter.drawLine(startScreen, endScreen);
            
            // Draw segment marker (small circle at segment end)
            if (violatesBendAngle) {
                painter.setBrush(QBrush(Qt::darkYellow));
                painter.setPen(QPen(Qt::darkYellow, 2));
            } else if (isDeviationSegment) {
                painter.setBrush(QBrush(Qt::red));
                painter.setPen(QPen(Qt::red, 2));
            } else {
                painter.setBrush(QBrush(Qt::darkGreen));
                painter.setPen(QPen(Qt::darkGreen, 1));
            }
            painter.drawEllipse(endScreen, 4, 4);
            
            // Draw violation indicators for problematic segments
            if (violatesBendAngle) {
                QPoint centerScreen = worldToScreen(segment.startPoint);
                
                // Draw bend angle violation warning (diamond shape)
                QPolygonF diamond;
                diamond << QPointF(centerScreen.x(), centerScreen.y() - 8)
                      << QPointF(centerScreen.x() + 6, centerScreen.y())
                      << QPointF(centerScreen.x(), centerScreen.y() + 8)
                      << QPointF(centerScreen.x() - 6, centerScreen.y());
                
                painter.setBrush(QBrush(Qt::darkYellow));
                painter.setPen(QPen(Qt::darkYellow, 1));
                painter.drawPolygon(diamond);
                
                // Bend angle violation indicator (no text)
            } else if (isDeviationSegment) {
                QPoint centerScreen = worldToScreen(segment.startPoint);
                
                // Draw deviation warning triangle
                QPolygonF triangle;
                triangle << QPointF(centerScreen.x(), centerScreen.y() - 8)
                       << QPointF(centerScreen.x() - 6, centerScreen.y() + 4)
                       << QPointF(centerScreen.x() + 6, centerScreen.y() + 4);
                
                painter.setBrush(QBrush(Qt::red));
                painter.setPen(QPen(Qt::red, 1));
                painter.drawPolygon(triangle);
                
                // Deviation indicator (no text)
            }
        }
        
        // Draw bend radius indicators where segments change direction significantly
        for (size_t i = 1; i < m_solverSegments.size(); ++i) {
            const auto& prevSegment = m_solverSegments[i-1];
            const auto& currentSegment = m_solverSegments[i];
            
            QVector3D prevDir = normalizeVector(prevSegment.endPoint - prevSegment.startPoint);
            QVector3D currentDir = normalizeVector(currentSegment.endPoint - currentSegment.startPoint);
            
            double dot = dotProduct(prevDir, currentDir);
            dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
            double angle = std::acos(dot);
            
            if (angle > 0.1) { // Significant direction change
                QPoint centerScreen = worldToScreen(prevSegment.endPoint);
                
                // Check if this waypoint is part of a deviation segment or has bend angle violations
                bool isDeviationWaypoint = false;
                bool hasBendAngleViolation = false;
                
                if (i < m_segmentDeviations.size()) {
                    isDeviationWaypoint = m_segmentDeviations[i];
                }
                
                // Check for bend angle violation at this waypoint
                if (i < m_solverSegments.size()) {
                    const auto& currentSegment = m_solverSegments[i];
                    if (currentSegment.isBend && currentSegment.bendRadius > 0.0) {
                        double maxBendAngle = 2.0 * std::asin(m_pitchLength / (2.0 * m_bendRadius));
                        hasBendAngleViolation = currentSegment.bendAngle > maxBendAngle;
                    }
                }
                
                // Draw bend radius indicator with appropriate color (bend angle violations take priority)
                if (hasBendAngleViolation) {
                    painter.setPen(QPen(Qt::darkYellow, 2, Qt::SolidLine));
                    painter.setBrush(QBrush(Qt::darkYellow));
                } else if (isDeviationWaypoint) {
                    painter.setPen(QPen(Qt::red, 2, Qt::SolidLine));
                    painter.setBrush(QBrush(Qt::red));
                } else {
                    painter.setPen(QPen(Qt::blue, 2, Qt::SolidLine));
                    painter.setBrush(Qt::NoBrush);
                }
                painter.drawEllipse(centerScreen, 8, 8);
                
                // Draw angle indicator
                painter.setPen(Qt::black);
                painter.setFont(QFont("Arial", 7));
                painter.drawText(centerScreen + QPoint(10, -10), 
                               QString("%1°").arg(static_cast<int>(angle * 180.0 / M_PI)));
            }
        }
        
        // Draw end point deviation indicator if the final waypoint doesn't match the target
        if (!m_solverSegments.empty()) {
            const auto& lastSegment = m_solverSegments.back();
            double finalDistanceToTarget = (lastSegment.endPoint - m_endPoint).length();
            if (finalDistanceToTarget > m_deviationThreshold) {
                QPoint endScreen = worldToScreen(lastSegment.endPoint);
                QPoint targetScreen = worldToScreen(m_endPoint);
                
                // Draw line from actual end to target end
                painter.setPen(QPen(Qt::red, 2, Qt::DashLine));
                painter.drawLine(endScreen, targetScreen);
                
                // Draw target end point with different color
                painter.setPen(QPen(Qt::red, 3));
                painter.setBrush(QBrush(Qt::red));
                painter.drawEllipse(targetScreen, 8, 8);
                
                // Draw deviation text
                painter.setPen(Qt::red);
                painter.setFont(QFont("Arial", 9, QFont::Bold));
                QString deviationText = QString("MISS: %1mm").arg(finalDistanceToTarget, 0, 'f', 1);
                painter.drawText(targetScreen + QPoint(15, -15), deviationText);
                
                // Draw arrow indicating direction of deviation
                QVector3D deviationVector = m_endPoint - lastSegment.endPoint;
                QVector3D arrowDirection = normalizeVector(deviationVector);
                QPoint arrowStart = worldToScreen(lastSegment.endPoint);
                QPoint arrowEnd = worldToScreen(lastSegment.endPoint + arrowDirection * 20.0);
                
                // Draw arrow
                painter.setPen(QPen(Qt::red, 2, Qt::SolidLine));
                painter.drawLine(arrowStart, arrowEnd);
                
                // Draw arrowhead
                QVector3D perpendicular = normalizeVector(QVector3D::crossProduct(arrowDirection, QVector3D(0, 0, 1)));
                QPoint arrowHead1 = worldToScreen(lastSegment.endPoint + arrowDirection * 20.0 + perpendicular * 5.0);
                QPoint arrowHead2 = worldToScreen(lastSegment.endPoint + arrowDirection * 20.0 - perpendicular * 5.0);
                painter.drawLine(arrowEnd, arrowHead1);
                painter.drawLine(arrowEnd, arrowHead2);
            }
        }
    } else {
        // Fallback to constraint-respecting path if no solver segments available
        qDebug() << "[2DVisualization] No solver segments available, using constraint-respecting path";
    
    // Calculate maximum bend angle allowed by the bend radius constraint
    double maxBendAngle = 2.0 * std::asin(m_pitchLength / (2.0 * m_bendRadius));
    
    // Generate constraint-respecting waypoints
    std::vector<QVector3D> waypoints;
    waypoints.push_back(m_startPoint);
    
    for (size_t i = 0; i < curvePoints.size() - 2; ++i) {
        QVector3D p0 = curvePoints[i];
        QVector3D p1 = curvePoints[i + 1];
        QVector3D p2 = curvePoints[i + 2];
        
        // Generate constraint-respecting waypoints for this segment
        auto segmentWaypoints = generateConstraintRespectingWaypoints(p0, p2, p1, m_pitchLength, maxBendAngle);
        
        // Add waypoints (skip first if it's the same as the last waypoint)
        for (size_t j = 1; j < segmentWaypoints.size(); ++j) {
            if (waypoints.empty() || (segmentWaypoints[j] - waypoints.back()).length() > 0.1) {
                waypoints.push_back(segmentWaypoints[j]);
            }
        }
    }
    
        // Draw the constraint-respecting segments with deviation highlighting
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        QPoint startScreen = worldToScreen(waypoints[i]);
        QPoint endScreen = worldToScreen(waypoints[i + 1]);
            
            // Check if this segment causes deviation
            bool isDeviationSegment = false;
            double distanceToTarget = 0.0;
            
            if (i < m_segmentDeviations.size()) {
                isDeviationSegment = m_segmentDeviations[i];
            }
            
            if (i < m_segmentDistancesToTarget.size()) {
                distanceToTarget = m_segmentDistancesToTarget[i];
            }
            
            // Set color based on deviation status
            if (isDeviationSegment) {
                // Red for segments that cause deviation
                painter.setPen(QPen(Qt::red, 4, Qt::SolidLine));
            } else {
                // Green for normal segments
                painter.setPen(QPen(Qt::darkGreen, 3, Qt::SolidLine));
            }
            
        painter.drawLine(startScreen, endScreen);
        
        // Draw segment marker (small circle at segment end)
            if (isDeviationSegment) {
                painter.setBrush(QBrush(Qt::red));
                painter.setPen(QPen(Qt::red, 2));
            } else {
        painter.setBrush(QBrush(Qt::darkGreen));
                painter.setPen(QPen(Qt::darkGreen, 1));
            }
        painter.drawEllipse(endScreen, 4, 4);
            
            // Draw deviation indicator for problematic segments
            if (isDeviationSegment) {
                QPoint centerScreen = worldToScreen(waypoints[i]);
                
                // Draw warning triangle
                QPolygonF triangle;
                triangle << QPointF(centerScreen.x(), centerScreen.y() - 8)
                       << QPointF(centerScreen.x() - 6, centerScreen.y() + 4)
                       << QPointF(centerScreen.x() + 6, centerScreen.y() + 4);
                
                painter.setBrush(QBrush(Qt::red));
                painter.setPen(QPen(Qt::red, 1));
                painter.drawPolygon(triangle);
                
                // Draw deviation text - REMOVED
                // painter.setPen(Qt::red);
                // painter.setFont(QFont("Arial", 8, QFont::Bold));
                // QString deviationText = QString("Δ%1mm").arg(distanceToTarget, 0, 'f', 1);
                // painter.drawText(centerScreen + QPoint(10, -10), deviationText);
            }
    }
    
    // Draw bend radius indicators where segments change direction significantly
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        QVector3D prevDir = normalizeVector(waypoints[i] - waypoints[i-1]);
        QVector3D nextDir = normalizeVector(waypoints[i+1] - waypoints[i]);
        
                    double dot = dotProduct(prevDir, nextDir);
            dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
            double angle = std::acos(dot);
        
        if (angle > 0.1) { // Significant direction change
            QPoint centerScreen = worldToScreen(waypoints[i]);
            
                // Check if this waypoint is part of a deviation segment
                bool isDeviationWaypoint = false;
                if (i < m_segmentDeviations.size()) {
                    isDeviationWaypoint = m_segmentDeviations[i];
                }
                
                // Draw bend radius indicator with appropriate color
                if (isDeviationWaypoint) {
            painter.setPen(QPen(Qt::red, 2, Qt::SolidLine));
                    painter.setBrush(QBrush(Qt::red));
                } else {
                    painter.setPen(QPen(Qt::blue, 2, Qt::SolidLine));
            painter.setBrush(Qt::NoBrush);
                }
            painter.drawEllipse(centerScreen, 8, 8);
            
            // Draw angle indicator
                painter.setPen(Qt::black);
                painter.setFont(QFont("Arial", 7));
            painter.drawText(centerScreen + QPoint(10, -10), 
                           QString("%1°").arg(static_cast<int>(angle * 180.0 / M_PI)));
            }
        }
        
        // Draw end point deviation indicator if the final waypoint doesn't match the target
        if (!waypoints.empty()) {
            double finalDistanceToTarget = (waypoints.back() - m_endPoint).length();
            if (finalDistanceToTarget > m_deviationThreshold) {
                QPoint endScreen = worldToScreen(waypoints.back());
                QPoint targetScreen = worldToScreen(m_endPoint);
                
                // Draw line from actual end to target end
                painter.setPen(QPen(Qt::red, 2, Qt::DashLine));
                painter.drawLine(endScreen, targetScreen);
                
                // Draw target end point with different color
                painter.setPen(QPen(Qt::red, 3));
                painter.setBrush(QBrush(Qt::red));
                painter.drawEllipse(targetScreen, 8, 8);
                
                // Draw deviation text
                painter.setPen(Qt::red);
                painter.setFont(QFont("Arial", 9, QFont::Bold));
                QString deviationText = QString("MISS: %1mm").arg(finalDistanceToTarget, 0, 'f', 1);
                painter.drawText(targetScreen + QPoint(15, -15), deviationText);
                
                // Draw arrow indicating direction of deviation
                QVector3D deviationVector = m_endPoint - waypoints.back();
                QVector3D arrowDirection = normalizeVector(deviationVector);
                QPoint arrowStart = worldToScreen(waypoints.back());
                QPoint arrowEnd = worldToScreen(waypoints.back() + arrowDirection * 20.0);
                
                // Draw arrow
                painter.setPen(QPen(Qt::red, 2, Qt::SolidLine));
                painter.drawLine(arrowStart, arrowEnd);
                
                // Draw arrowhead
                QVector3D perpendicular = normalizeVector(QVector3D::crossProduct(arrowDirection, QVector3D(0, 0, 1)));
                QPoint arrowHead1 = worldToScreen(waypoints.back() + arrowDirection * 20.0 + perpendicular * 5.0);
                QPoint arrowHead2 = worldToScreen(waypoints.back() + arrowDirection * 20.0 - perpendicular * 5.0);
                painter.drawLine(arrowEnd, arrowHead1);
                painter.drawLine(arrowEnd, arrowHead2);
            }
        }
    }
}

// Helper functions for Bézier curve calculations
double DragChain2DVisualization::calculateBezierArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double tStart, double tEnd)
{
    // Simple numerical integration for arc length
    const int numSamples = 100;
    double arcLength = 0.0;
    
    for (int i = 0; i < numSamples; ++i) {
        double t1 = tStart + (tEnd - tStart) * i / numSamples;
        double t2 = tStart + (tEnd - tStart) * (i + 1) / numSamples;
        
        QVector3D point1 = evaluateBezierCurve(p0, p1, p2, t1);
        QVector3D point2 = evaluateBezierCurve(p0, p1, p2, t2);
        
        arcLength += (point2 - point1).length();
    }
    
    return arcLength;
}

double DragChain2DVisualization::findBezierParameterForArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double targetLength, double tStart)
{
    // Binary search to find t parameter for target arc length
    double tMin = tStart;
    double tMax = 1.0;
    double tolerance = 0.001;
    
    while (tMax - tMin > tolerance) {
        double tMid = (tMin + tMax) / 2.0;
        double arcLength = calculateBezierArcLength(p0, p1, p2, tStart, tMid);
        
        if (arcLength < targetLength) {
            tMin = tMid;
        } else {
            tMax = tMid;
        }
    }
    
    return (tMin + tMax) / 2.0;
}

QVector3D DragChain2DVisualization::evaluateBezierCurve(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double t)
{
    // Quadratic Bézier curve evaluation
    return (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * p1 + t * t * p2;
}

// Helper functions for constraint-respecting path generation
std::vector<QVector3D> DragChain2DVisualization::generateConstraintRespectingWaypoints(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendAngle)
{
    std::vector<QVector3D> waypoints;
    waypoints.push_back(startPoint);
    
    // Calculate total arc length of the Bézier curve
    double totalArcLength = calculateBezierArcLength(startPoint, controlPoint, endPoint, 0.0, 1.0);
    int segmentCount = static_cast<int>(std::ceil(totalArcLength / pitchLength));
    
    // Ensure at least 2 segments for a meaningful path
    if (segmentCount < 2) segmentCount = 2;
    
    // Define the plane using start, end, and control points
    QVector3D planeNormal = normalizeVector(QVector3D::crossProduct(endPoint - startPoint, controlPoint - startPoint));
    if (planeNormal.length() < 0.001) {
        // If the three points are collinear, use a default plane normal
        planeNormal = QVector3D(0, 0, 1);
    }
    
    // Helper function to project a point onto the plane
    auto projectToPlane = [&](const QVector3D& point) -> QVector3D {
        QVector3D pointToStart = point - startPoint;
        double distanceFromPlane = QVector3D::dotProduct(pointToStart, planeNormal);
        return point - planeNormal * distanceFromPlane;
    };
    
    // Generate waypoints that respect the bend angle constraint
    QVector3D currentPoint = startPoint;
    QVector3D currentDirection = normalizeVector(controlPoint - startPoint);
    
    for (int i = 1; i <= segmentCount; ++i) {
        // Calculate target position along the Bézier curve
        double t = static_cast<double>(i) / segmentCount;
        QVector3D targetPoint = evaluateBezierCurve(startPoint, controlPoint, endPoint, t);
        
        // Calculate the ideal direction to the target
        QVector3D idealDirection = normalizeVector(targetPoint - currentPoint);
        
        // Check if the angle change exceeds the maximum allowed bend angle
                    double dot = dotProduct(currentDirection, idealDirection);
            dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
            double angleChange = std::acos(dot);
        
        if (angleChange > maxBendAngle && i < segmentCount) {
            // Need to create an intermediate waypoint to respect the bend angle constraint
            
            // Calculate the maximum allowed direction change
            QVector3D rotationAxis = normalizeVector(crossProduct(currentDirection, idealDirection));
            if (rotationAxis.length() < 0.001) {
                // Vectors are parallel, no rotation needed
                rotationAxis = QVector3D(0, 0, 1);
            }
            
            // Ensure rotation axis is in the plane
            rotationAxis = normalizeVector(QVector3D::crossProduct(planeNormal, rotationAxis));
            
            // Rotate current direction by the maximum allowed angle
            QVector3D constrainedDirection = rotateVector(currentDirection, rotationAxis, maxBendAngle);
            
            // Calculate intermediate waypoint
            QVector3D intermediatePoint = currentPoint + constrainedDirection * pitchLength;
            
            // Project the waypoint onto the plane to ensure it stays in the intended plane
            intermediatePoint = projectToPlane(intermediatePoint);
            
            waypoints.push_back(intermediatePoint);
            
            // Update for next iteration
            currentPoint = intermediatePoint;
            currentDirection = normalizeVector(intermediatePoint - waypoints[waypoints.size() - 2]);
        } else {
            // Angle change is acceptable, move directly to target
            currentPoint = targetPoint;
            currentDirection = idealDirection;
        }
    }
    
    // Ensure the last waypoint is exactly the end point
    waypoints.back() = endPoint;
    
    return waypoints;
}

QVector3D DragChain2DVisualization::normalizeVector(const QVector3D& vector)
{
    double length = vector.length();
    if (length < 0.001) return QVector3D(0, 0, 0);
    return vector / length;
}

QVector3D DragChain2DVisualization::crossProduct(const QVector3D& a, const QVector3D& b)
{
    return QVector3D::crossProduct(a, b);
}

double DragChain2DVisualization::dotProduct(const QVector3D& a, const QVector3D& b)
{
    return QVector3D::dotProduct(a, b);
}

double ConnectionCreationWidget::safeClamp(double value, double min, double max) const
{
    return (value < min) ? min : (value > max) ? max : value;
}

QVector3D DragChain2DVisualization::rotateVector(const QVector3D& vector, const QVector3D& axis, double angle)
{
    // Rodrigues' rotation formula
    QVector3D unitAxis = normalizeVector(axis);
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    
    return vector * cosAngle + 
           crossProduct(unitAxis, vector) * sinAngle + 
           unitAxis * dotProduct(unitAxis, vector) * (1.0 - cosAngle);
}

void ConnectionCreationWidget::testFixedSegmentLengths()
{
    qDebug() << "[ConnectionWidget] Testing fixed segment lengths...";
    
    if (!m_point1 || !m_point2) {
        qDebug() << "[ConnectionWidget] No points selected for testing";
        return;
    }
    
    // Get the world positions
    QVector3D point1(0, 0, 0); // Placeholder - in real implementation, get from model
    QVector3D point2(100, 50, 0); // Placeholder - in real implementation, get from model
    
    double pitchLength = m_pitchLengthSpin->value();
    double maxBendRadius = m_maxBendRadiusSpin->value();
    
    qDebug() << "[ConnectionWidget] Test parameters:";
    qDebug() << "[ConnectionWidget]   Point1:" << point1;
    qDebug() << "[ConnectionWidget]   Point2:" << point2;
    qDebug() << "[ConnectionWidget]   Pitch length:" << pitchLength;
    qDebug() << "[ConnectionWidget]   Max bend radius:" << maxBendRadius;
    
    // Create solver and test
    BezierDragChainSolver solver;
    auto segments = solver.solveDragChainPathSimple(point1, point2, (point1 + point2) * 0.5f, pitchLength, maxBendRadius);
    
    qDebug() << "[ConnectionWidget] Generated" << segments.size() << "segments";
    
    // Verify all segments have exactly the pitch length
    bool allCorrect = true;
    for (size_t i = 0; i < segments.size(); ++i) {
        double segmentLength = (segments[i].endPoint - segments[i].startPoint).length();
        double difference = std::abs(segmentLength - pitchLength);
        
        qDebug() << "[ConnectionWidget] Segment" << i << ":";
        qDebug() << "[ConnectionWidget]   Length:" << segmentLength << "mm";
        qDebug() << "[ConnectionWidget]   Expected:" << pitchLength << "mm";
        qDebug() << "[ConnectionWidget]   Difference:" << difference << "mm";
        
        if (difference > 0.001) {
            qDebug() << "[ConnectionWidget]   ERROR: Segment length is not fixed!";
            allCorrect = false;
        }
    }
    
    if (allCorrect) {
        qDebug() << "[ConnectionWidget] ✓ All segments have exactly the pitch length!";
    } else {
        qDebug() << "[ConnectionWidget] ✗ Some segments have incorrect lengths!";
    }
}

void ConnectionCreationWidget::testDeviationFlagging()
{
    qDebug() << "[ConnectionWidget] Testing deviation flagging functionality...";
    
    if (!m_point1 || !m_point2) {
        qDebug() << "[ConnectionWidget] No points selected for testing";
        return;
    }
    
    // Create test deviation data
    std::vector<bool> testDeviations = {false, true, false, true, false};
    std::vector<double> testDistances = {50.0, 45.2, 40.1, 35.8, 30.5};
    
    qDebug() << "[ConnectionWidget] Test deviation data:";
    for (size_t i = 0; i < testDeviations.size(); ++i) {
        qDebug() << "[ConnectionWidget]   Segment" << i << ":" << (testDeviations[i] ? "DEVIATES" : "normal") << "distance:" << testDistances[i] << "mm";
    }
    
    // Pass test data to 2D visualization
    if (m_2dVisualization) {
        m_2dVisualization->setDeviationThreshold(5.0); // 5mm threshold
        m_2dVisualization->setDeviationData(testDeviations, testDistances);
        qDebug() << "[ConnectionWidget] ✓ Test deviation data passed to 2D visualization";
    } else {
        qDebug() << "[ConnectionWidget] ✗ No 2D visualization available for testing";
    }
}

void DragChain2DVisualization::setDeviationThreshold(double threshold)
{
    m_deviationThreshold = threshold;
    update();
}

void DragChain2DVisualization::setDeviationData(const std::vector<bool>& segmentDeviations, 
                                               const std::vector<double>& segmentDistancesToTarget)
{
    m_segmentDeviations = segmentDeviations;
    m_segmentDistancesToTarget = segmentDistancesToTarget;
    update();
}

void DragChain2DVisualization::setActualPathSegments(const std::vector<QVector3D>& pathWaypoints)
{
    m_actualPathWaypoints = pathWaypoints;
    update();
}

void DragChain2DVisualization::setSolverSegments(const std::vector<BezierDragChainSegment>& solverSegments)
{
    m_solverSegments = solverSegments;
    update();
}