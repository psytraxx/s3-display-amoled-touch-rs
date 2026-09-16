# Changelog

All notable changes to this project are documented here. Entries are grouped by date and by type (Added, Changed, Fixed, Removed). Each entry explains *why* the change was made and *what* effect it has, not how it was implemented.

## Unreleased

### Fixed

- The Radar tab is now hidden when no LD2410 sensor is connected, instead of showing a UI page that would never receive data. Previously an unresponsive/missing radar sensor would still show a Radar screen with no way to know it wasn't working.
- Removed deprecated Slint layout properties that were producing build warnings, keeping the UI on a supported API surface going forward.
