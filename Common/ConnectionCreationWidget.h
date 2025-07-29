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
#include "CadNode.h"
#include "CustomModelTreeModel.h"
#include "CadOpenGLWidget.h"
#include "DragChainConstraintSolver.h"

// Forward declarations
class DragChainConstraintSolver;

class ConnectionCreationWidget : public QWidget {
    Q_OBJECT
public:
    explicit ConnectionCreationWidget(CustomModelTreeModel* model, QWidget* parent = nullptr);
    
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
        qDebug() << "[ConnectionWidget] clearSolverStatistics called - clearing all statistics";
        m_lastSolverStatistics = StoredSolverStatistics(); 
    }
    
    // Set the OpenGL widget for visualization
    void setOpenGLWidget(CadOpenGLWidget* openGLWidget);
    
    // Update connection path visualization
    void updateConnectionPathVisualization();
    
    // Update solver statistics display
    void updateSolverStatisticsDisplay();
    
    // Handle real-time solver statistics updates
    void onSolverStatisticsUpdated(const DragChainConstraintSolver::SolverStatistics& stats);
    
    // Check if solver statistics are valid
    bool hasValidSolverStatistics() const;

signals:
    void connectionCreated(std::shared_ptr<CadNode> connectionNode);
    void connectionCancelled();

private slots:
    void onPoint1SelectionChanged();
    void onPoint2SelectionChanged();
    void onConnectionTypeChanged(int index);
    void onAttachmentConfigChanged();
    void onUpdateVisualization();
    void onCreateConnection();
    void onCancel();

private:
    void setupUI();
    void populateConnectionPoints();
    void updateConnectionInfo();
    bool validateConnection();
    
    // Helper methods for attachment configuration
    QVector3D getPlaneNormalFromIndex(int index) const;
    QVector3D getDirectionFromIndex(int index) const;
    
    // Unified drag chain path creation
    std::vector<CadOpenGLWidget::ConnectionPathSegment> createDragChainPath(
        const QVector3D& point1, const QVector3D& point2, const QVector3D& direction, 
        double distance, double bendRadius, double pitchLength, int segmentCount,
        bool startLocked, bool endLocked);
    
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
    QSpinBox* m_segmentCountSpin;
    
    // Solver parameter controls
    QSpinBox* m_maxIterationsSpin;
    QDoubleSpinBox* m_convergenceToleranceSpin;
    QDoubleSpinBox* m_simulationTimeStepSpin;
    
    // Attachment configuration UI elements
    QGroupBox* m_startAttachmentGroup;
    QGroupBox* m_endAttachmentGroup;
    QCheckBox* m_lockStartAttachmentCheck;
    QCheckBox* m_lockEndAttachmentCheck;
    QComboBox* m_startPlaneCombo;
    QComboBox* m_endPlaneCombo;
    QComboBox* m_startDirectionCombo;
    QComboBox* m_endDirectionCombo;
    
    QLabel* m_distanceLabel;
    QLabel* m_calculatedLengthLabel;
    QLabel* m_effectiveLengthLabel;
    QLabel* m_compatibilityLabel;
    QLabel* m_placementLabel;
    QLabel* m_segmentLengthsLabel;
    QLabel* m_solverIterationsLabel;
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
        double minSegmentLength;
        double maxSegmentLength;
        double averageSegmentLength;
        
        StoredSolverStatistics() : iterationsUsed(0), finalDistanceToTarget(0.0), converged(false), 
                                 simulationTime(0.0), rigidBodiesCreated(0), jointsCreated(0),
                                 minSegmentLength(0.0), maxSegmentLength(0.0), averageSegmentLength(0.0) {}
    };
    StoredSolverStatistics m_lastSolverStatistics;
};

#endif // CONNECTIONCREATIONWIDGET_H 
