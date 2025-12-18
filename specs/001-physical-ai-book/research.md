# Research: Simulation Platforms and VLA Models

**Date**: 2025-12-16
**Status**: Completed

## 1. Evaluation of Gazebo vs. Unity for Digital Twin Simulation

**Task**: "Evaluate Gazebo vs. Unity for digital twin simulation in the context of ROS 2 and NVIDIA Isaac Sim."

### Decision
The book will primarily use **Gazebo** for initial ROS 2 simulations and then transition to **NVIDIA Isaac Sim** for high-fidelity digital twins and advanced topics. Unity will be mentioned as an alternative but not used for the core labs.

### Rationale
- **Gazebo**: It is the de-facto standard for ROS 2 simulation. It's open-source, tightly integrated with the ROS ecosystem, and provides a solid foundation for understanding basic simulation concepts. This makes it the ideal starting point for students.
- **NVIDIA Isaac Sim**: It offers photorealistic rendering, advanced physics simulation (PhysX), and native integration with NVIDIA's AI stack (including Isaac ROS). It represents the state-of-the-art for creating digital twins and is the logical next step for advanced lessons.
- **Unity**: While powerful and also supported by NVIDIA, focusing on two simulation platforms (Gazebo for fundamentals, Isaac Sim for advanced) provides a clearer learning path. Introducing a third would add unnecessary complexity.

### Alternatives Considered
- **Using only Gazebo**: This would limit the book's ability to teach advanced digital twin concepts and photorealistic simulation.
- **Using only Isaac Sim**: This might present a steeper learning curve for beginners who are new to both ROS 2 and simulation.
- **Using Unity instead of Isaac Sim**: Isaac Sim has a more direct and streamlined integration with the NVIDIA AI stack and ROS, which is a core focus of this book.

---

## 2. Identification of Vision-Language-Action (VLA) Models

**Task**: "Identify suitable open-source or easily accessible Vision-Language-Action (VLA) models for the capstone project."

### Decision
The capstone project will leverage a pre-trained, open-source VLA model such as **Llava (Large Language and Vision Assistant)** or a similar model available on Hugging Face. The focus will be on the *integration* of such a model into a ROS 2 system, not on training a VLA from scratch.

### Rationale
- **Accessibility**: Using a popular, pre-trained model like Llava ensures that students can complete the capstone without requiring massive computational resources for training.
- **Focus on Integration**: The primary learning objective is to understand how to build an end-to-end embodied AI system. The VLA is a component in this system. Integrating a pre-built model is a more realistic and achievable goal for the scope of this book.
- **Extensibility**: Students can be guided on how they could fine-tune the model or swap it out for another one, providing a path for further exploration.

### Alternatives Considered
- **Training a custom VLA model**: This is a PhD-level task and far beyond the scope of this book.
- **Using a proprietary, closed-source model**: This would violate the principle of using accessible, open tools where possible and could introduce costs or access limitations for students.

---

## 3. Best Practices for Docusaurus Project Structure

**Task**: "Find best practices for structuring a Docusaurus project with a large number of modules and lessons."

### Decision
The Docusaurus site will be structured using its built-in "docs" versioning and multi-instance features if needed, but will start with a simple file-based hierarchy. Content will be organized in subdirectories within the `/docs` folder, matching the module structure. The sidebar will be automatically generated from this structure using `autogenerate`.

### Rationale
- **Simplicity**: Starting with a simple, file-based hierarchy is the easiest approach and is well-supported by Docusaurus.
- **Scalability**: Docusaurus's sidebar autogeneration (`sidebar.js`) is powerful enough to handle many modules and nested lessons without manual configuration.
- **Official Recommendation**: This approach aligns with the official Docusaurus documentation and community best practices.

### Alternatives Considered
- **Manual Sidebar Configuration**: This would be tedious and error-prone as the book grows.
- **Docs Versioning from the start**: This adds complexity that isn't needed until there is a second version of the book.
