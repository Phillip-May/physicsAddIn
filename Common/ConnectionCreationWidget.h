#ifndef CONNECTIONCREATIONWIDGET_H
#define CONNECTIONCREATIONWIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>
#include <QFrame>
#include <set>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include "CadNode.h"
#include "CustomModelTreeModel.h"
#include "CadOpenGLWidget.h"
#include "BezierDragChainSolver.h"

// Forward declarations
class BezierDragChainSolver;

// 2D Visualization Widget for Drag Chain Control Points
class DragChain2DVisualization : public QFrame {
    Q_OBJECT
public:
    explicit DragChain2DVisualization(QWidget* parent = nullptr);
    
    void setStartPoint(const QVector3D& point);
    void setEndPoint(const QVector3D& point);
    void setControlPoints(const std::vector<QVector3D>& points);
    void setPitchLength(double pitchLength);
    void setBendRadius(double bendRadius);
    void clear();
    
    // Deviation tracking methods
    void setDeviationThreshold(double threshold);
    void setDeviationData(const std::vector<bool>& segmentDeviations, 
                         const std::vector<double>& segmentDistancesToTarget);
    
    // Actual path segments from solver
    void setActualPathSegments(const std::vector<QVector3D>& pathWaypoints);
    void setSolverSegments(const std::vector<BezierDragChainSegment>& solverSegments);
    
    // Getter methods
    QVector3D getStartPoint() const { return m_startPoint; }
    QVector3D getEndPoint() const { return m_endPoint; }
    
signals:
    void startPointChanged(const QVector3D& point);
    void endPointChanged(const QVector3D& point);
    void controlPointChanged(int index, const QVector3D& point);
    void controlPointAdded(int index, const QVector3D& point);
    void controlPointRemoved(int index);
    
protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    
private:
    QVector3D m_startPoint;
    QVector3D m_endPoint;
    std::vector<QVector3D> m_controlPoints;
    double m_pitchLength;
    double m_bendRadius;
    bool m_hasStartPoint;
    bool m_hasEndPoint;
    
    // Deviation tracking
    double m_deviationThreshold;
    std::vector<bool> m_segmentDeviations;
    std::vector<double> m_segmentDistancesToTarget;
    
    // Actual solver path data
    std::vector<QVector3D> m_actualPathWaypoints;
    std::vector<BezierDragChainSegment> m_solverSegments;
    
    // Mouse interaction
    int m_selectedPointIndex;
    bool m_dragging;
    QPoint m_lastMousePos;
    
    // Coordinate transformation
    QPoint worldToScreen(const QVector3D& worldPos) const;
    QVector3D screenToWorld(const QPoint& screenPos) const;
    void updateTransform();
    
    // Drawing helpers
    void drawGrid(QPainter& painter);
    void drawPoints(QPainter& painter);
    void drawBezierCurve(QPainter& painter);
    void drawSegments(QPainter& painter);
    
    // Bézier curve helper functions
    double calculateBezierArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double tStart, double tEnd);
    double findBezierParameterForArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double targetLength, double tStart);
    QVector3D evaluateBezierCurve(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double t);
    
    // Constraint-respecting path generation
    std::vector<QVector3D> generateConstraintRespectingWaypoints(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,
        double pitchLength,
        double maxBendAngle);
    
    // Vector math helper functions
    QVector3D normalizeVector(const QVector3D& vector);
    QVector3D crossProduct(const QVector3D& a, const QVector3D& b);
    double dotProduct(const QVector3D& a, const QVector3D& b);
    QVector3D rotateVector(const QVector3D& vector, const QVector3D& axis, double angle);
    
    // Transform data
    QTransform m_transform;
    QRectF m_worldBounds;
    QRect m_screenBounds;
};

class ConnectionCreationWidget : public QWidget {
    Q_OBJECT
public:
    explicit ConnectionCreationWidget(CustomModelTreeModel* model, QWidget* parent = nullptr);
    ~ConnectionCreationWidget();

    // JSON serialization methods
    QJsonObject saveSettingsToJson() const;
    bool loadSettingsFromJson(const QJsonObject& settings);
    void saveSettingsToFile(const QString& filename) const;
    bool loadSettingsFromFile(const QString& filename);
    
    // Convenience methods
    void saveSettingsToDefaultFile() const;
    bool loadSettingsFromDefaultFile();
    QString getSettingsAsString() const;
    
    // Load settings from a connection node
    bool loadSettingsFromConnectionNode(const std::shared_ptr<CadNode>& connectionNode);

    // Get the created connection node
    std::shared_ptr<CadNode> getConnectionNode() const { return m_connectionNode; }
    
    // Get selected connection points
    std::shared_ptr<CadNode> getPoint1() const { return m_point1; }
    std::shared_ptr<CadNode> getPoint2() const { return m_point2; }
    
    // Check if connection was created successfully
    bool isConnectionCreated() const { return m_connectionNode != nullptr; }
    
    // Clear the widget state
    void clearState();
    
        // Clear solver statistics
    void clearSolverStatistics() { 
        qDebug() << "[ConnectionWidget] clearSolverStatistics called - clearing all statistics and segments";
        m_lastSolverStatistics = StoredSolverStatistics(); 
        m_solverSegments.clear(); // Also clear stored segments to force re-run
        updateSolverStatisticsDisplay();
    }
    
    // Clear user modifications to control points
    void clearUserControlPointModifications() {
        qDebug() << "[ConnectionWidget] clearUserControlPointModifications called";
        m_userModifiedControlPointIndices.clear();
        m_userModifiedControlPoint = false;
    }
    
    // Set the OpenGL widget for visualization
    void setOpenGLWidget(CadOpenGLWidget* openGLWidget);
    
    // Update connection path visualization
    void updateConnectionPathVisualization();
    
    // Update solver statistics display
    void updateSolverStatisticsDisplay();
    
    // Handle real-time solver statistics updates
    void onSolverStatisticsUpdated(const BezierDragChainSolver::SolverStatistics& stats);
    
    // Check if solver statistics are valid
    bool hasValidSolverStatistics() const;

signals:
    void connectionCreated(std::shared_ptr<CadNode> connectionNode);
    void connectionCancelled();

private slots:
    void onPoint1SelectionChanged();
    void onPoint2SelectionChanged();
    void onConnectionTypeChanged(int index);
    void onUpdateVisualization();
    void onCreateConnection();
    void onCancel();
    void onStartPointChanged(const QVector3D& point);
    void onEndPointChanged(const QVector3D& point);
    void onControlPointChanged(int index, const QVector3D& point);
    void onControlPointAdded(int index, const QVector3D& point);
    void onControlPointRemoved(int index);
    void onDeviationThresholdChanged(double value);
    void onChainDimensionsChanged(double value);
    
    // Test function for fixed segment lengths
    void testFixedSegmentLengths();
    void testDeviationFlagging();
    
private:
    void setupUI();
    void populateConnectionPoints();
    void updateConnectionInfo();
    bool validateConnection();
    
    // Helper method to project a point to the 2D plane defined by start and end points
    QVector3D projectTo2DPlane(const QVector3D& point, const QVector3D& startPoint, const QVector3D& endPoint) const;
    
    // Manual clamp function to avoid std::clamp issues
    double safeClamp(double value, double min, double max) const;
    
    // Unified drag chain path creation
    std::vector<CadOpenGLWidget::ConnectionPathSegment> createDragChainPath(
        const QVector3D& point1, const QVector3D& point2, const QVector3D& direction, 
        double distance, double bendRadius, double pitchLength);
    
    CustomModelTreeModel* m_model;
    std::shared_ptr<CadNode> m_connectionNode;
    std::shared_ptr<CadNode> m_point1;
    std::shared_ptr<CadNode> m_point2;
    
    // UI elements
    QListWidget* m_point1List;
    QListWidget* m_point2List;
    QComboBox* m_connectionTypeCombo;
    QLineEdit* m_connectionNameEdit;

    QCheckBox* m_isFlexibleCheck;
    QDoubleSpinBox* m_maxBendRadiusSpin;
    QDoubleSpinBox* m_pitchLengthSpin;
    
    // Bézier curve solver parameters
    QDoubleSpinBox* m_optimizationStepSpin;
    QDoubleSpinBox* m_curveSmoothnessSpin;
    QDoubleSpinBox* m_deviationThresholdSpin;
    QDoubleSpinBox* m_chainWidthSpin;
    QDoubleSpinBox* m_chainHeightSpin;
    
    // Segment count is now always calculated automatically based on pitch length
    
    // Store current control points for visualization (auto-calculated only)
    std::vector<QVector3D> m_currentControlPoints;
    
    // Store solver segments for 2D visualization
    std::vector<BezierDragChainSegment> m_solverSegments;
    
    // Track if user has manually modified the auto-calculated control point
    bool m_userModifiedControlPoint;
    
    // Track which specific control points have been modified by the user
    std::set<int> m_userModifiedControlPointIndices;
    
    // 2D Visualization
    DragChain2DVisualization* m_2dVisualization;
    
    QLabel* m_compatibilityLabel;
    QLabel* m_segmentLengthsLabel;
    QLabel* m_solverIterationsLabel;
    QLabel* m_cableLengthLabel;
    QLabel* m_segmentCountLabel;
    QLabel* m_segmentAnglesLabel;
    QPushButton* m_createButton;
    QPushButton* m_cancelButton;
    QPushButton* m_updateVisualizationButton;
    
    // Store connection points separately to avoid Qt meta-object system
    std::vector<std::shared_ptr<CadNode>> m_availablePoints;
    
    // Store references to original nodes for finding locations in tree
    std::vector<CadNode*> m_originalConnectionPoints;
    
    // OpenGL widget for visualization
    CadOpenGLWidget* m_openGLWidget = nullptr;
    
    // Store last solver statistics
    struct StoredSolverStatistics {
        int iterationsUsed;
        double finalDistanceToTarget;
        bool converged;
        double simulationTime;
        int rigidBodiesCreated;
        int jointsCreated;
        int controlPointsOptimized;  // Store actual segment count from solver
        double minSegmentLength;
        double maxSegmentLength;
        double averageSegmentLength;
        double minSegmentAngle;
        double maxSegmentAngle;
        double averageSegmentAngle;
        double totalCableLength;
        int totalSegments;
        
        StoredSolverStatistics() : iterationsUsed(0), finalDistanceToTarget(0.0), converged(false), 
                                 simulationTime(0.0), rigidBodiesCreated(0), jointsCreated(0),
                                 controlPointsOptimized(0), minSegmentLength(0.0), maxSegmentLength(0.0), averageSegmentLength(0.0),
                                 minSegmentAngle(0.0), maxSegmentAngle(0.0), averageSegmentAngle(0.0),
                                 totalCableLength(0.0), totalSegments(0) {}
    };
    StoredSolverStatistics m_lastSolverStatistics;
};

#endif // CONNECTIONCREATIONWIDGET_H 
