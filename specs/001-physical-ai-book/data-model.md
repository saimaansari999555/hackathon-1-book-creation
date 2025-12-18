# Data Model: Book Content Structure

**Date**: 2025-12-16
**Status**: Draft

This document defines the structure and relationships of the content entities for the "Physical AI & Humanoid Robotics Technical Book". This is a conceptual model, not a database schema.

## Entity Relationship Diagram (Conceptual)

```
[Book] 1--* [Module]
  |
  '-- [Capstone Project]

[Module] 1--* [Weekly Lesson]

[Weekly Lesson] 1--* [Lab]
  |
  '--* [Simulation]
```

## Entity Definitions

### 1. Book

The top-level entity representing the entire technical book.

- **Attributes**:
    - `title`: The title of the book (e.g., "Physical AI & Humanoid Robotics").
    - `author`: The author(s) of the book.
    - `version`: The version of the book.
- **Relationships**:
    - Has many `Modules`.
    - Has one `Capstone Project`.

### 2. Module

A major section of the book, focused on a high-level topic.

- **Attributes**:
    - `title`: The title of the module (e.g., "ROS 2 Fundamentals", "NVIDIA Isaac Sim").
    - `order`: The numerical order in which the module appears.
    - `summary`: A brief description of what the module covers.
- **Relationships**:
    - Belongs to one `Book`.
    - Has many `Weekly Lessons`.

### 3. Weekly Lesson

A smaller, digestible unit of learning within a module.

- **Attributes**:
    - `title`: The title of the lesson (e.g., "Understanding ROS 2 Nodes", "Creating a URDF Model").
    - `order`: The numerical order within the module.
    - `content`: The core instructional text and images (in Markdown).
- **Relationships**:
    - Belongs to one `Module`.
    - Can have many `Labs`.
    - Can have many `Simulations`.

### 4. Lab

A hands-on, step-by-step exercise designed to reinforce the concepts from a lesson.

- **Attributes**:
    - `title`: The title of the lab (e.g., "Lab: Building Your First Publisher/Subscriber").
    - `objective`: A clear statement of what the user will achieve.
    - `instructions`: The step-by-step guide.
    - `code_path`: A link to the associated source code in the `/labs` directory.
- **Relationships**:
    - Belongs to one `Weekly Lesson`.

### 5. Simulation

A pre-configured simulation environment used for a lab or demonstration.

- **Attributes**:
    - `name`: The name of the simulation (e.g., "empty_world_gazebo").
    - `platform`: The simulation platform used (`Gazebo` or `Isaac Sim`).
    - `description`: A brief description of the environment and its purpose.
    - `launch_command`: The command to run the simulation.
- **Relationships**:
    - Can belong to one or more `Weekly Lessons` or `Labs`.

### 6. Capstone Project

A final, integrative project that requires the user to apply skills from all modules.

- **Attributes**:
    - `title`: "Capstone Project: Autonomous Voice-Activated Humanoid".
    - `problem_statement`: A description of the challenge.
    - `high_level_steps`: An outline of the major steps to complete the project.
    - `final_criteria`: The criteria for a successful project completion.
- **Relationships**:
    - Belongs to one `Book`.
