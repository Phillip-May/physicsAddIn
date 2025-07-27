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
#include "MaterialEditorDialog.h"
#include "ObjectPropertiesDialog.h"
#include "CadViewerDialog.h"

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
    RDK = rdk;
    MainWindow = mw;
    StatusBar = statusbar;
    
    // Initialize physics engine
    
    // Initialize managers
    
    // Set the object properties manager in the physics engine
    
    qDebug() << "Loading plugin " << PluginName();
    qDebug() << "Using settings: " << settings; // reserved for future compatibility

    // it is highly recommended to use the statusbar for debugging purposes (pass /DEBUG as an argument to see debug result in RoboDK)
    qDebug() << "Setting up the status bar";
    StatusBar->showMessage(tr("RoboDK Plugin %1 is being loaded").arg(PluginName()));

    // Add scene configuration action
    actionSceneConfig = new QAction(tr("Scene Defaults"), this);
    connect(actionSceneConfig, &QAction::triggered, this, [this] {

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

    // Add CAD viewer action
    actionCadViewer = new QAction(tr("CAD Viewer"), this);
    connect(actionCadViewer, &QAction::triggered, this, [this] {

        showCadViewer();

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
    menu1->addAction(actionCadViewer);

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
    if (actionCadViewer != nullptr){
        actionCadViewer->deleteLater();
        actionCadViewer = nullptr;
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
        // Create a checkbox action for objects
        QAction* physXAction = menu->addAction("PhysX Simulation");
        physXAction->setCheckable(true);

        connect(physXAction, &QAction::toggled, this, [this, item](bool checked) {

            if (checked) {
                // Add object to PhysX simulation
                qDebug() << "Adding object to PhysX simulation";
            } else {
                // Remove object from PhysX simulation
                qDebug() << "Removing object from PhysX simulation";
            }

        }, Qt::QueuedConnection);
        
        // Add material selection submenu for objects in simulation
        menu->addSeparator();

        QMenu* materialMenu = menu->addMenu("Material");

        // Add available materials
        QStringList materials = m_materialManager->getAvailableMaterials();
        for (const QString& materialName : materials) {
            QAction* materialAction = materialMenu->addAction(materialName);
            materialAction->setCheckable(true);

            connect(materialAction, &QAction::triggered, this, [this, item, materialName]() {
            }, Qt::QueuedConnection);
        }
        
        // Add object properties option for objects in simulation
        menu->addSeparator();
        QAction* propertiesAction = menu->addAction("Object Properties");
        connect(propertiesAction, &QAction::triggered, this, [this, item]() {
            showObjectProperties(item);
        }, Qt::QueuedConnection);
    }
    
    if (item->Type() == IItem::ITEM_TYPE_ROBOT) {
        // Create a checkbox action for robots
        QAction* physXAction = menu->addAction("PhysX Robot Simulation");
        physXAction->setCheckable(true);

        connect(physXAction, &QAction::toggled, this, [this, item](bool checked) {

            if (checked) {
                // Add robot to PhysX simulation
                qDebug() << "Adding robot to PhysX simulation";

            } else {
                // Remove robot from PhysX simulation
                qDebug() << "Removing robot from PhysX simulation";
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
        break;
    case EventMoved:
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
    if (!item) {
        // This version is called from the menu action without a specific item
        // We could show a dialog to select an object, or just show a message
        QMessageBox::information(MainWindow, "Object Properties", 
                               "Please right-click on an object in the scene to edit its properties.");
        return;
    }

}

void PluginPhysics::showSoftBodyDialog()
{       

}

void PluginPhysics::showCadViewer()
{
    CadViewerDialog* dialog = new CadViewerDialog(MainWindow);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->show();
}
