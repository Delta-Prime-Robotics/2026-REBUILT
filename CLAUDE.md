# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Java-based FIRST Robotics Competition (FRC) robot codebase for Team 6328, built on WPILib 2026. The robot uses a swerve drive configuration with a climber subsystem. The project follows a command-based architecture with comprehensive logging and simulation support via AdvantageKit.

## Essential Commands

### Build and Deploy
```bash
# Build the project
./gradlew build

# Deploy to robot (automatically formats code via Spotless)
./gradlew deploy

# Run simulation
./gradlew simulate
```

### Testing and Code Quality
```bash
# Run tests
./gradlew test

# Format code (also runs automatically during build)
./gradlew spotlessApply
```

### AdvantageKit Log Replay
```bash
# Watch and replay AdvantageKit logs
./gradlew replayWatch
```

### Event Branches
When deploying from a branch starting with "event", Gradle automatically creates a git commit with all changes before deploying. This is useful for competition events.

## Architecture Overview

### IO Pattern (Hardware Abstraction)

The codebase uses a critical IO pattern for hardware abstraction that separates logic from hardware implementation. Each subsystem has:
- **IO Interface** (e.g., `GyroIO.java`, `ModuleIO.java`): Defines what the subsystem needs from hardware
- **Implementation Classes**:
  - Real hardware implementations (e.g., `ModuleIOSparkMax`)
  - Simulation implementations (e.g., `ModuleIOSim`)
  - Replay implementations (for log replay)

This pattern enables seamless testing, simulation, and log replay. When adding new subsystems, you must maintain this separation.

### Subsystem Structure

Subsystems are organized in `src/main/java/frc/robot/subsystems/`:
- **drive/**: Swerve drive implementation with module IO abstraction
- **climber/**: HookSubsystem (currently being ported to REVLib 2025+)

Each subsystem typically contains:
- Main subsystem class (e.g., `Drive.java`, `HookSubsystem.java`)
- Constants class (e.g., `DriveConstants.java`)
- IO interfaces and implementations

### Robot Container Pattern

`RobotContainer.java` is the central configuration hub that:
- Binds subsystems to commands
- Sets up autonomous routines
- Configures button bindings
- Initializes the AdvantageKit Logger

### Commands

Commands are stored in `src/main/java/frc/robot/commands/`. The codebase uses inline lambdas for simple commands rather than separate command classes.

## Key Libraries and Versions

- **AdvantageKit 26.0.0**: Advanced logging and replay system
- **PathPlannerLib**: Autonomous path following (paths in `src/main/deploy/pathplanner/`)
- **REVLib 2025+**: Modern REV hardware API with configuration classes
- **PhotonLib**: Vision processing
- **URCL**: Underlying Robot Control Library for hardware diagnostics

## Hardware Configuration

- **Team Number**: 6328
- **Drive CAN IDs**: 1-9 (reserved for swerve drive motors)
- **Climber CAN ID**: 18
- Operating modes: `REAL`, `SIM`, `REPLAY` (controlled by AdvantageKit)

## Code Style and Formatting

The project uses **Spotless** with Google Java Format for consistent code style. Formatting is automatically applied during compilation. BSD license headers are required on all source files.

## Adding New Subsystems

When creating a new subsystem:
1. Create subsystem directory under `subsystems/`
2. Define IO interface for hardware abstraction
3. Create real hardware implementation (e.g., using REVLib for Spark Max motors)
4. Create simulation implementation (if applicable)
5. Create constants file for configuration values
6. Wire up in `RobotContainer.java`
7. Ensure AdvantageKit `@AutoLog` annotations are used for logged inputs/outputs

## REVLib 2025+ Notes

When working with REV hardware:
- Use the new configuration-based API (e.g., `SparkMaxConfig`)
- Follow patterns in `ModuleIOSparkMax.java` for reference
- Always apply configurations using `applyConfig()` or `configure()`
- The climber subsystem is currently being migrated to this new API

## Simulation and Replay

The project supports full AdvantageKit log replay. The sim GUI is disabled by default to support replay. All hardware changes should be logged through the IO pattern to enable accurate replay.
