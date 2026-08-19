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


# CLAUDE.md - Project Guide for AI Assistants

## Project Overview

**FIRST Agentic CSA** is an MCP (Model Context Protocol) server that provides intelligent documentation search across FIRST Robotics Competition (FRC) documentation. It aggregates docs from multiple vendors (WPILib, REV Robotics, CTRE Phoenix, Redux, PhotonVision) and enables natural language queries via AI assistants.

## Tech Stack

- **Language:** Python 3.11+
- **Package Manager:** uv (Astral's Rust-based manager)
- **Protocol:** MCP 2025-06-18 schema
- **Search:** BM25 ranking via `rank-bm25`
- **HTTP:** httpx with async support
- **HTML Parsing:** BeautifulSoup4 + lxml
- **Build:** Hatchling
- **Testing:** pytest with asyncio

## Project Structure

```
src/wpilib_mcp/
├── server.py           # MCP server entry point & tool registration
├── tool_router.py      # Routes tool calls to plugins
├── plugin_loader.py    # Plugin discovery & loading
├── plugins/            # Vendor plugin implementations
│   ├── base.py         # PluginBase abstract class
│   ├── wpilib/         # WPILib core docs
│   ├── rev/            # REV Robotics (SparkMax)
│   ├── ctre/           # CTRE Phoenix (TalonFX)
│   ├── redux/          # Redux Robotics
│   └── photonvision/   # PhotonVision
└── utils/
    ├── search.py       # BM25 search indexing
    ├── fetch.py        # HTTP fetching with caching
    ├── html.py         # HTML cleaning & extraction
    └── indexer.py      # Index management

tests/                  # pytest test suite
scripts/                # Utility scripts (index building)
config.json             # Runtime configuration
server.json             # MCP server manifest
```

## Common Commands

```bash
# Install dependencies
uv sync --all-extras

# Run tests
uv run pytest tests/ -v

# Run server locally
uv run first-agentic-csa

# Build package
uv build

# Build documentation indexes
python scripts/build_index.py all
python scripts/build_index.py wpilib --version 2025
python scripts/build_index.py rev
```

## Architecture Patterns

### Plugin System
- Each vendor implements a Plugin class extending `PluginBase`
- Plugins are loaded dynamically based on `config.json`
- Lifecycle: load → initialize → search/fetch → shutdown

### Async Pattern
All I/O operations are async:
```python
async def search(query, version, language, max_results) -> list[SearchResult]
async def fetch_page(url) -> Optional[PageContent]
```

### Key Data Classes
```python
@dataclass
class SearchResult:
    url, title, section, vendor, language, version, content_preview, score

@dataclass
class PageContent:
    url, title, content, vendor, language, version, section, last_fetched
```

### MCP Tools Exposed
1. `search_frc_docs` - Multi-vendor search with filters
2. `fetch_frc_doc_page` - Full page content retrieval
3. `list_frc_doc_sections` - Browse available documentation

## Coding Conventions

- Use async/await for all I/O operations
- Type hints on all function signatures
- Dataclasses for structured data
- Graceful error handling (don't crash the server)
- Pre-built JSON indexes stored in each plugin's `data/` directory

## Configuration

`config.json` controls:
- Plugin enable/disable
- Supported languages per vendor (Java, Python, C++)
- Supported versions (2024, 2025)
- Cache TTL and size limits
- Default search parameters

## Testing

```bash
# Run all tests
uv run pytest tests/ -v

# Run specific test file
uv run pytest tests/test_plugins.py -v
uv run pytest tests/test_search.py -v
```

## Entry Point

The package exposes `first-agentic-csa` command which runs `wpilib_mcp.server:main()`
