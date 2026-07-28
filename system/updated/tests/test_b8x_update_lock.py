from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
LAUNCH_ENV_PATH = ROOT / "launch_env.sh"
LAUNCHER_PATH = ROOT / "launch_chffrplus.sh"
MANAGER_PATH = ROOT / "system" / "manager" / "manager.py"
PROCESS_CONFIG_PATH = ROOT / "system" / "manager" / "process_config.py"
UPDATED_PATH = ROOT / "system" / "updated" / "updated.py"
HARDWARED_PATH = ROOT / "system" / "hardware" / "hardwared.py"


class TestB8xUpdateLock(unittest.TestCase):
  def test_update_lock_is_exported_by_the_installed_branch(self):
    launch_env_source = LAUNCH_ENV_PATH.read_text(encoding="utf-8")
    self.assertIn("export DISABLE_AUTO_UPDATES=1", launch_env_source)

  def test_launcher_blocks_even_an_already_staged_update(self):
    launcher_source = LAUNCHER_PATH.read_text(encoding="utf-8")
    policy_check = 'if [ "${DISABLE_AUTO_UPDATES:-0}" = "1" ]; then'
    staged_install = 'if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then'
    self.assertIn(policy_check, launcher_source)
    self.assertIn(staged_install, launcher_source)
    self.assertLess(
      launcher_source.index(policy_check),
      launcher_source.index(staged_install),
    )

  def test_manager_persists_disable_state_and_clears_stale_prompt(self):
    manager_source = MANAGER_PATH.read_text(encoding="utf-8")
    self.assertIn(
      'if os.getenv("DISABLE_AUTO_UPDATES") == "1":',
      manager_source,
    )
    self.assertIn('params.put_bool("DisableUpdates", True)', manager_source)
    self.assertIn('params.put_bool("UpdateAvailable", False)', manager_source)

  def test_process_supervisor_keeps_updater_stopped(self):
    process_source = PROCESS_CONFIG_PATH.read_text(encoding="utf-8")
    self.assertIn(
      'return not started and not params.get_bool("DisableUpdates")',
      process_source,
    )
    self.assertIn(
      'PythonProcess("updated", "system.updated.updated", updater, enabled=not PC)',
      process_source,
    )

  def test_independent_updater_and_startup_guards_remain(self):
    updated_source = UPDATED_PATH.read_text(encoding="utf-8")
    hardwared_source = HARDWARED_PATH.read_text(encoding="utf-8")
    self.assertIn(
      'if params.get_bool("DisableUpdates"):',
      updated_source,
    )
    self.assertIn(
      'params.get_bool("DisableUpdates")',
      hardwared_source,
    )


if __name__ == "__main__":
  unittest.main()
