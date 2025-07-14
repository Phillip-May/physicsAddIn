# Physics Simulation Plugin for RoboDK

This plugin adds physics simulation capabilities to RoboDK, including rigid body dynamics, soft body simulation, and material management.

## What it does

- **Rigid body physics**: Uses PhysX for realistic object interactions
- **Soft body simulation**: Deformable objects that respond to forces
- **Material system**: Customize friction, restitution, and other properties
- **Object configuration**: Fine-tune how objects behave in simulation
- **Scene setup**: Configure global physics settings and defaults
- **VHACD integration**: Automatic convex decomposition for complex shapes
- **Real-time visualization**: See physics in action
- **Robot support**: Include robots in your physics simulations

## Getting started

### Requirements
- RoboDK (download from [robodk.com](https://robodk.com/download))
- PhysX SDK (included with the plugin)

### Installation
Plugins typically come as RoboDK Packages (.rdkp) that install automatically when opened in RoboDK.

For manual installation, copy the plugin files to your RoboDK plugins folder (usually `C:/RoboDK/bin/plugins`).

### Setup
1. Enable the plugin (Tools → Add-ins or press Shift+I)
2. Look for "Physics Simulation" in the menu
3. Right-click objects to add them to physics simulation
4. Configure materials and properties as needed

## How to use it

### Adding objects to physics
- Right-click any object in your scene
- Select "PhysX Simulation" to add/remove from physics
- Objects get default materials automatically

### Working with materials
- Go to "Physics Simulation" → "Material Manager"
- Create custom materials with specific properties
- Edit existing materials (default ones are read-only)

### Creating soft bodies
- Use "Physics Simulation" → "Create Soft Body"
- Adjust soft body settings
- Choose between new geometry or existing objects

### Scene configuration
- Access via "Physics Simulation" → "Scene Defaults"
- Set up global physics parameters
- Configure default materials and properties

## About RoboDK Plugins

The RoboDK Plugin Interface lets you extend and customize RoboDK with native C++ plugins that integrate directly into the core software.

You can add custom functionality to the RoboDK interface, including new menu items, toolbar buttons, event processing, render synchronization, API command handling, and more.

Once your plugin is complete, you can package it as a self-contained .rdkp file for easy distribution.

RoboDK handles plugins through the Add-in Manager, and the [Plugin Interface](https://github.com/RoboDK/Plug-In-Interface) provides the C++ tools needed to build your plugin.

For more information, check out the [Plugin Interface GitHub](https://github.com/RoboDK/Plug-In-Interface) and the [documentation](https://robodk.com/doc/en/PlugIns/index.html). 