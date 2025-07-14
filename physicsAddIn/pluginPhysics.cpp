#include "robodktools.h"
#include "irobodk.h"
#include "iitem.h"

#include <QMainWindow>
#include <QToolBar>
#include <QDebug>
#include <QAction>
#include <QStatusBar>
#include <QMenuBar>
#include <QTextEdit>
#include <QDateTime>
#include <QIcon>
#include <QDesktopServices>
#include <QTcpServer>
#include <QTimer>
#include <QApplication>
#include <QCoreApplication>
#include <QStyleFactory>
#include <QWindow>
#include <QElapsedTimer>
#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QMessageBox>
#include <QMenu>

#include <fstream>
#include <iostream>
#include <vector>

#include "pluginPhysics.h"
#include "PhysXEngine.h"  // For LoadBinarySTL, CreateConvexMesh, convertSTLToPhysX
#include "MaterialEditorDialog.h"
#include "ObjectPropertiesDialog.h"
#include "SoftBodyConfigDialog.h"

// VHACD Library - header only (implementation in PhysXEngine.cpp)
#include "VHACD.h"

using namespace physx;

// Function prototypes for functions used before definition

QString PluginPhysics::getPluginName() {
    return "Physics Simulation";
}

QString PluginPhysics::PluginName(){
    return getPluginName();
}

static QElapsedTimer *elapsedTimer;
bool isRenderDone = true;

QString PluginPhysics::PluginLoad(QMainWindow *mw, QMenuBar *menubar, QStatusBar *statusbar, RoboDK *rdk, const QString &settings){
    PhysXEngine::setRoboDK(rdk);
    PhysXEngine::setLoadingDone(false);
    RDK = rdk;
    MainWindow = mw;
    StatusBar = statusbar;
    
    // Initialize physics engine
    m_physicsEngine = new PhysXEngine(rdk, this);
    if (!m_physicsEngine->initialize()) {
        qDebug() << "Failed to initialize physics engine";
    }
    
    // Initialize managers
    m_materialManager = new MaterialManager(m_physicsEngine, this);
    m_objectPropertiesManager = new ObjectPropertiesManager(m_physicsEngine, m_materialManager, this);
    
    // Set the object properties manager in the physics engine
    m_physicsEngine->setObjectPropertiesManager(m_objectPropertiesManager);
    m_sceneConfigDialog = new SceneConfigurationDialog(m_physicsEngine, m_materialManager, mw);
    
    qDebug() << "Loading plugin " << PluginName();
    qDebug() << "Using settings: " << settings; // reserved for future compatibility

    // it is highly recommended to use the statusbar for debugging purposes (pass /DEBUG as an argument to see debug result in RoboDK)
    qDebug() << "Setting up the status bar";
    StatusBar->showMessage(tr("RoboDK Plugin %1 is being loaded").arg(PluginName()));

    // Add scene configuration action
    actionSceneConfig = new QAction(tr("Scene Defaults"), this);
    connect(actionSceneConfig, &QAction::triggered, this, [this] {

        if (m_sceneConfigDialog) {
            m_sceneConfigDialog->show();
        }

    }, Qt::QueuedConnection);

    // Add material management action
    actionMaterialManager = new QAction(tr("Material Manager"), this);
    connect(actionMaterialManager, &QAction::triggered, this, [this] {

        showMaterialManager();

    }, Qt::QueuedConnection);

    // Add soft body creation action
    actionCreateSoftBody = new QAction(tr("Create Soft Body"), this);
    connect(actionCreateSoftBody, &QAction::triggered, this, [this] {

        showSoftBodyDialog();

    }, Qt::QueuedConnection);

    // Here you can add one or more actions in the menu
    qDebug() << "Setting up the menu bar";

    QMenu *menu1 = mw->findChild<QMenu *>("menu-Program");
    if (menu1 == nullptr){
        menu1 = menubar->addMenu(tr("Physics Simulation"));
    }
    // Add actions to the menu
    menu1->addAction(actionSceneConfig);
    menu1->addAction(actionMaterialManager);
    menu1->addAction(actionCreateSoftBody);

    // For triggering render updates because RoboDK won't do that otherwise
    QTimer* frameTimer = new QTimer(this);
    //This actually triggers crashes in robodk, so for now just hope that things
    //Move in the scene often enough
    frameTimer->start(16);

    // Ensure the timer's slot is always executed on the main (GUI) thread
    // Because otherwise robodk will crash
    QObject::connect(frameTimer, &QTimer::timeout, this, [this]() {
        // This code runs every frame, on the main thread  
        qDebug() << "Force Render";
        RDK->Render(RoboDK::RenderComplete);
        if (isRenderDone) {
            isRenderDone = false;
        }
    }, Qt::QueuedConnection);

    PhysXEngine::setLoadingDone(true);

    // Test VHACD integration
    testVHACDIntegration();

    // return string is reserverd for future compatibility
    return "";
}


void PluginPhysics::PluginUnload(){
    // Cleanup the plugin
    qDebug() << "Unloading plugin " << PluginName();
    
    // Clean up managers
    if (m_materialManager) {
        delete m_materialManager;
        m_materialManager = nullptr;
    }
    
    if (m_objectPropertiesManager) {
        delete m_objectPropertiesManager;
        m_objectPropertiesManager = nullptr;
    }
    
    // Clean up scene configuration dialog
    if (m_sceneConfigDialog) {
        delete m_sceneConfigDialog;
        m_sceneConfigDialog = nullptr;
    }
    
    // Clean up physics engine
    if (m_physicsEngine) {
        m_physicsEngine->cleanup();
        delete m_physicsEngine;
        m_physicsEngine = nullptr;
    }
    
    // Physics cleanup is now handled by PhysXEngine

    if (actionSceneConfig != nullptr){
        actionSceneConfig->deleteLater();
        actionSceneConfig = nullptr;
    }
    if (actionMaterialManager != nullptr){
        actionMaterialManager->deleteLater();
        actionMaterialManager = nullptr;
    }
    if (actionCreateSoftBody != nullptr){
        actionCreateSoftBody->deleteLater();
        actionCreateSoftBody = nullptr;
    }
}

void PluginPhysics::PluginLoadToolbar(QMainWindow *mw, int icon_size){
    // TODO: Add toolbar functionality if needed
}

bool PluginPhysics::PluginItemClick(Item item, QMenu *menu, TypeClick click_type){
    if (click_type != ClickRight){
        return false;
    }

    if (item->Type() == IItem::ITEM_TYPE_OBJECT) {
        // Check if this object is already in the PhysX simulation
        bool isInPhysX = m_physicsEngine ? m_physicsEngine->isObjectInSimulation(item) : false;

        // Create a checkbox action for objects
        QAction* physXAction = menu->addAction("PhysX Simulation");
        physXAction->setCheckable(true);
        physXAction->setChecked(isInPhysX);

        connect(physXAction, &QAction::toggled, this, [this, item](bool checked) {

            if (checked) {
                // Add object to PhysX simulation
                qDebug() << "Adding object to PhysX simulation";

                // Use the physics engine
                if (m_physicsEngine && m_physicsEngine->addObject(item)) {
                    qDebug() << "Successfully added object using PhysXEngine";
                    
                    // Automatically assign Wood material to new objects
                    if (m_materialManager) {
                        m_materialManager->setObjectMaterial(item, "Wood");
                        qDebug() << "Automatically assigned Wood material to new object";
                    }
                } else {
                    qDebug() << "Failed to add object using PhysXEngine";
                }
            } else {
                // Remove object from PhysX simulation
                qDebug() << "Removing object from PhysX simulation";

                // Use the physics engine
                if (m_physicsEngine && m_physicsEngine->removeObject(item)) {
                    qDebug() << "Successfully removed object using PhysXEngine";
                } else {
                    qDebug() << "Failed to remove object using PhysXEngine";
                }
            }

        }, Qt::QueuedConnection);
        
        // Add material selection submenu for objects in simulation
        if (isInPhysX && m_materialManager) {
            menu->addSeparator();
            
            QMenu* materialMenu = menu->addMenu("Material");
            
            // Get current material
            QString currentMaterial = m_materialManager->getObjectMaterial(item);
            
            // Add available materials
            QStringList materials = m_materialManager->getAvailableMaterials();
            for (const QString& materialName : materials) {
                QAction* materialAction = materialMenu->addAction(materialName);
                materialAction->setCheckable(true);
                materialAction->setChecked(currentMaterial == materialName);
                
                connect(materialAction, &QAction::triggered, this, [this, item, materialName]() {

                    m_materialManager->setObjectMaterial(item, materialName);

                }, Qt::QueuedConnection);
            }
        }
        
        // Add object properties option for objects in simulation
        if (isInPhysX) {
            menu->addSeparator();
            QAction* propertiesAction = menu->addAction("Object Properties");
            connect(propertiesAction, &QAction::triggered, this, [this, item]() {
                showObjectProperties(item);
            }, Qt::QueuedConnection);
        }
    }
    
    if (item->Type() == IItem::ITEM_TYPE_ROBOT) {
        // Check if this robot is already in the PhysX simulation
        bool isInPhysX = m_physicsEngine ? m_physicsEngine->isRobotInSimulation(item) : false;

        // Create a checkbox action for robots
        QAction* physXAction = menu->addAction("PhysX Robot Simulation");
        physXAction->setCheckable(true);
        physXAction->setChecked(isInPhysX);

        connect(physXAction, &QAction::toggled, this, [this, item](bool checked) {

            if (checked) {
                // Add robot to PhysX simulation
                qDebug() << "Adding robot to PhysX simulation";

                // Use the physics engine
                if (m_physicsEngine && m_physicsEngine->addRobot(item)) {
                    qDebug() << "Successfully added robot using PhysXEngine";
                } else {
                    qDebug() << "Failed to add robot using PhysXEngine";
                }
            } else {
                // Remove robot from PhysX simulation
                qDebug() << "Removing robot from PhysX simulation";

                // Use the physics engine
                if (m_physicsEngine && m_physicsEngine->removeRobot(item)) {
                    qDebug() << "Successfully removed robot using PhysXEngine";
                } else {
                    qDebug() << "Failed to remove robot using PhysXEngine";
                }
            }

        }, Qt::QueuedConnection);
    }

    if (item->Type() != IItem::ITEM_TYPE_STATION){
        return false;
    }

    return true;
}

QString PluginPhysics::PluginCommand(const QString &command, const QString &value){
    // TODO: Implement plugin commands if needed
    return "";
}

void PluginPhysics::PluginEvent(TypeEvent event_type){
    switch (event_type) {
    case EventRender:
        // Render debug visualization
        if (m_physicsEngine) {
            m_physicsEngine->renderDebugGeometry();
        }
        break;
    case EventMoved:
    {
        if (elapsedTimer == nullptr) {
            elapsedTimer = new QElapsedTimer();
            elapsedTimer->start();
        }
        if (!PhysXEngine::isLoadingDone()) {
            return;
        }

        qint64 elapsed = elapsedTimer->nsecsElapsed();
        float deltaTime = elapsed / 1000000000.0f;
        elapsedTimer->restart();

        if (deltaTime == 0) {
            return;
        }

        // Use the physics engine for extended simulation step
        if (m_physicsEngine) {
            m_physicsEngine->stepSimulationExtended(deltaTime);
        }
        isRenderDone = true;
    }
        break;
    case EventChanged:
        break;
    case EventAbout2Save:
        break;
    case EventAbout2ChangeStation:
    case EventAbout2CloseStation:
        // TODO: Handle station changes if needed
        break;
    case EventTrajectoryStep:
        break;
    default:
        qDebug() << "Unknown/future event: " << event_type;
    }
}

void PluginPhysics::showMaterialManager()
{
    if (!m_materialManager) {
        qWarning() << "Material manager not initialized";
        return;
    }
    
    // Create a simple dialog to show available materials and allow adding custom ones
    QDialog dialog(MainWindow);
    dialog.setWindowTitle("Material Manager");
    dialog.setFixedSize(500, 400);
    
    QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);
    
    // Available materials list
    QLabel* materialsLabel = new QLabel("Available Materials:");
    mainLayout->addWidget(materialsLabel);
    
    QListWidget* materialsList = new QListWidget();
    QStringList materials = m_materialManager->getAvailableMaterials();
    for (const QString& materialName : materials) {
        MaterialProperties material = m_materialManager->getMaterial(materialName);
        QString itemText = QString("%1 (Static: %2, Dynamic: %3, Restitution: %4)")
                          .arg(materialName)
                          .arg(material.staticFriction)
                          .arg(material.dynamicFriction)
                          .arg(material.restitution);
        
        QListWidgetItem* item = new QListWidgetItem(itemText);
        if (m_materialManager->isDefaultMaterial(materialName)) {
            item->setBackground(QColor(240, 240, 240)); // Light gray for default materials
        }
        materialsList->addItem(item);
    }
    mainLayout->addWidget(materialsList);
    
    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    
    QPushButton* addButton = new QPushButton("Add Custom Material");
    QPushButton* editButton = new QPushButton("Edit Material");
    QPushButton* removeButton = new QPushButton("Remove Material");
    QPushButton* closeButton = new QPushButton("Close");
    
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(editButton);
    buttonLayout->addWidget(removeButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(closeButton);
    
    mainLayout->addLayout(buttonLayout);
    
    // Connect signals
    connect(addButton, &QPushButton::clicked, this, [&]() {

        MaterialEditorDialog editor(&dialog);
        if (editor.exec() == QDialog::Accepted) {
            MaterialProperties newMaterial = editor.getMaterialProperties();
            if (m_materialManager->addCustomMaterial(newMaterial)) {
                // Refresh the list
                materialsList->clear();
                QStringList updatedMaterials = m_materialManager->getAvailableMaterials();
                for (const QString& materialName : updatedMaterials) {
                    MaterialProperties material = m_materialManager->getMaterial(materialName);
                    QString itemText = QString("%1 (Static: %2, Dynamic: %3, Restitution: %4)")
                                      .arg(materialName)
                                      .arg(material.staticFriction)
                                      .arg(material.dynamicFriction)
                                      .arg(material.restitution);
                    
                    QListWidgetItem* item = new QListWidgetItem(itemText);
                    if (m_materialManager->isDefaultMaterial(materialName)) {
                        item->setBackground(QColor(240, 240, 240));
                    }
                    materialsList->addItem(item);
                }
            }
        }

    }, Qt::QueuedConnection);
    
    connect(editButton, &QPushButton::clicked, this, [&]() {

        QListWidgetItem* currentItem = materialsList->currentItem();
        if (!currentItem) {
            QMessageBox::warning(&dialog, "No Selection", "Please select a material to edit.");
            return;
        }
        
        QString materialName = currentItem->text().split(" ").first();
        if (m_materialManager->isDefaultMaterial(materialName)) {
            QMessageBox::information(&dialog, "Default Material", 
                                   "Default materials cannot be edited. Create a custom material instead.");
            return;
        }
        
        MaterialProperties material = m_materialManager->getMaterial(materialName);
        MaterialEditorDialog editor(material, &dialog);
        if (editor.exec() == QDialog::Accepted) {
            MaterialProperties updatedMaterial = editor.getMaterialProperties();
            if (m_materialManager->updateMaterial(materialName, updatedMaterial)) {
                // Update the list item
                QString itemText = QString("%1 (Static: %2, Dynamic: %3, Restitution: %4)")
                                  .arg(materialName)
                                  .arg(updatedMaterial.staticFriction)
                                  .arg(updatedMaterial.dynamicFriction)
                                  .arg(updatedMaterial.restitution);
                currentItem->setText(itemText);
            }
        }

    }, Qt::QueuedConnection);
    
    connect(removeButton, &QPushButton::clicked, this, [&]() {

        QListWidgetItem* currentItem = materialsList->currentItem();
        if (!currentItem) {
            QMessageBox::warning(&dialog, "No Selection", "Please select a material to remove.");
            return;
        }
        
        QString materialName = currentItem->text().split(" ").first();
        if (m_materialManager->isDefaultMaterial(materialName)) {
            QMessageBox::information(&dialog, "Default Material", 
                                   "Default materials cannot be removed.");
            return;
        }
        
        if (QMessageBox::question(&dialog, "Confirm Removal", 
                                 QString("Are you sure you want to remove the material '%1'?").arg(materialName),
                                 QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
            if (m_materialManager->removeCustomMaterial(materialName)) {
                materialsList->takeItem(materialsList->row(currentItem));
            }
        }

    }, Qt::QueuedConnection);
    
    connect(closeButton, &QPushButton::clicked, this, [&dialog]() {

        dialog.accept();

    }, Qt::QueuedConnection);
    
    dialog.exec();
}

void PluginPhysics::showObjectProperties(Item item)
{
    if (!m_objectPropertiesManager) {
        qWarning() << "Object properties manager not initialized";
        return;
    }
    
    if (!item) {
        // This version is called from the menu action without a specific item
        // We could show a dialog to select an object, or just show a message
        QMessageBox::information(MainWindow, "Object Properties", 
                               "Please right-click on an object in the scene to edit its properties.");
        return;
    }
    
    // Create and show the object properties dialog
    ObjectPropertiesDialog dialog(item, m_objectPropertiesManager, m_materialManager, MainWindow);
    if (dialog.exec() == QDialog::Accepted) {
        qDebug() << "Object properties updated for" << item->Name();
    }
}

void PluginPhysics::showSoftBodyDialog()
{
    if (!m_physicsEngine) {
        qWarning() << "Physics engine not initialized";
        return;
    }
    
    // Create and show the soft body configuration dialog
    SoftBodyConfigDialog dialog(MainWindow);
    
    // Connect the dialog's signal to handle configuration acceptance
    connect(&dialog, &SoftBodyConfigDialog::configAccepted, this, [this](const SoftBodyConfig& config) {
        // Always ask user to select an existing object to use as the soft body
        Item softBodyItem = RDK->ItemUserPick("Select an object to convert to a soft body", IItem::ITEM_TYPE_OBJECT);
        if (!softBodyItem) {
            qWarning() << "No object selected for soft body creation";
            StatusBar->showMessage(tr("No object selected"), 3000);
            return;
        }
        
        // Add the soft body to the physics simulation
        if (m_physicsEngine->addSoftBody(softBodyItem, config)) {
            qDebug() << "Successfully created soft body:" << softBodyItem->Name();
            StatusBar->showMessage(tr("Soft body created successfully"), 3000);
        } else {
            qWarning() << "Failed to create soft body";
            StatusBar->showMessage(tr("Failed to create soft body"), 3000);
        }
    });
    
    // Show the dialog
    if (dialog.exec() == QDialog::Accepted) {
        qDebug() << "Soft body configuration dialog accepted";
    } else {
        qDebug() << "Soft body configuration dialog cancelled";
    }
}

void PluginPhysics::testVHACDIntegration()
{
    qDebug() << "Testing VHACD integration...";
    
    try {
        // Create a simple test mesh (a cube)
        std::vector<VHACD::Vertex> vertices;
        std::vector<VHACD::Triangle> triangles;
        
        // Cube vertices (8 vertices)
        vertices.push_back(VHACD::Vertex(-1.0, -1.0, -1.0)); // 0
        vertices.push_back(VHACD::Vertex( 1.0, -1.0, -1.0)); // 1
        vertices.push_back(VHACD::Vertex( 1.0,  1.0, -1.0)); // 2
        vertices.push_back(VHACD::Vertex(-1.0,  1.0, -1.0)); // 3
        vertices.push_back(VHACD::Vertex(-1.0, -1.0,  1.0)); // 4
        vertices.push_back(VHACD::Vertex( 1.0, -1.0,  1.0)); // 5
        vertices.push_back(VHACD::Vertex( 1.0,  1.0,  1.0)); // 6
        vertices.push_back(VHACD::Vertex(-1.0,  1.0,  1.0)); // 7
        
        // Cube triangles (12 triangles, 2 per face)
        // Front face
        triangles.push_back(VHACD::Triangle(0, 1, 2));
        triangles.push_back(VHACD::Triangle(0, 2, 3));
        // Back face
        triangles.push_back(VHACD::Triangle(4, 6, 5));
        triangles.push_back(VHACD::Triangle(4, 7, 6));
        // Left face
        triangles.push_back(VHACD::Triangle(0, 4, 5));
        triangles.push_back(VHACD::Triangle(0, 5, 1));
        // Right face
        triangles.push_back(VHACD::Triangle(2, 6, 7));
        triangles.push_back(VHACD::Triangle(2, 7, 3));
        // Top face
        triangles.push_back(VHACD::Triangle(3, 7, 4));
        triangles.push_back(VHACD::Triangle(3, 4, 0));
        // Bottom face
        triangles.push_back(VHACD::Triangle(1, 5, 6));
        triangles.push_back(VHACD::Triangle(1, 6, 2));
        
        // Create VHACD instance
        VHACD::IVHACD* vhacd = VHACD::CreateVHACD();
        
        if (vhacd) {
            // Set parameters
            VHACD::IVHACD::Parameters params;
            params.m_maxConvexHulls = 1;  // We expect 1 convex hull for a cube
            params.m_resolution = 1000000; // High resolution
            params.m_minimumVolumePercentErrorAllowed = 1.0; // 1% volume error allowed
            params.m_maxRecursionDepth = 10; // Maximum recursion depth
            params.m_shrinkWrap = true; // Shrinkwrap voxel positions to source mesh
            params.m_fillMode = VHACD::FillMode::FLOOD_FILL; // How to fill interior
            params.m_maxNumVerticesPerCH = 64; // Max vertices per convex hull
            params.m_asyncACD = true; // Run asynchronously
            params.m_minEdgeLength = 2; // Min edge length for recursion
            params.m_findBestPlane = false; // Experimental feature, keep false
            params.m_callback = nullptr;
            params.m_logger = nullptr;
            params.m_taskRunner = nullptr;
            
            // Compute convex decomposition
            bool success = vhacd->Compute(&vertices[0].mX, 
                                         vertices.size(), 
                                         &triangles[0].mI0, 
                                         triangles.size(), 
                                         params);
            
            if (success) {
                uint32_t numHulls = vhacd->GetNConvexHulls();
                qDebug() << "VHACD test successful! Generated" << numHulls << "convex hull(s)";
                
                // Get the first convex hull
                if (numHulls > 0) {
                    VHACD::IVHACD::ConvexHull hull;
                    if (vhacd->GetConvexHull(0, hull)) {
                        qDebug() << "Convex hull 0 has" << hull.m_points.size() << "points and" << hull.m_triangles.size() << "triangles";
                    }
                }
                
                StatusBar->showMessage(tr("VHACD integration test successful"), 3000);
            } else {
                qWarning() << "VHACD test failed - computation unsuccessful";
                StatusBar->showMessage(tr("VHACD test failed"), 3000);
            }
            
            // Clean up
            vhacd->Clean();
            vhacd->Release();
        } else {
            qWarning() << "VHACD test failed - could not create VHACD instance";
            StatusBar->showMessage(tr("VHACD test failed - creation error"), 3000);
        }
        
    } catch (const std::exception& e) {
        qWarning() << "VHACD test failed with exception:" << e.what();
        StatusBar->showMessage(tr("VHACD test failed with exception"), 3000);
    } catch (...) {
        qWarning() << "VHACD test failed with unknown exception";
        StatusBar->showMessage(tr("VHACD test failed with unknown error"), 3000);
    }
} 