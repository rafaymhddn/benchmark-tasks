"""Models that are not in Arena's object library, kept in the repo next to the task that uses them (pure Python).

Each entry is a USD wrapper (physics added: rigid body, collider) around a downloaded model. ``Asset(...,
asset=<name>)`` uses them like library assets: ``register()`` (sim side, called by ``rta.scene``) adds them to
Arena's asset registry, and ``docs/assets/bounds.json`` holds their sizes for ``rta.assets``.
"""

from __future__ import annotations

from rta import TASKS_DIR

LOCAL = {
    # Poly Haven, CC0 (https://polyhaven.com/a/<id>): lightbulb_01, lightbulb_led, pull_chain_light_socket
    "bulb_old_polyhaven": TASKS_DIR / "l01_bulb_change" / "assets" / "bulb_old.usda",
    "bulb_new_polyhaven": TASKS_DIR / "l01_bulb_change" / "assets" / "bulb_new.usda",
    "lamp_socket_polyhaven": TASKS_DIR / "l01_bulb_change" / "assets" / "lamp_socket.usda",
    # built by tasks/h06_bluetooth_pairing/assets/build.py (original geometry and artwork)
    "h06_speaker": TASKS_DIR / "h06_bluetooth_pairing" / "assets" / "speaker.usda",
    "h06_phone": TASKS_DIR / "h06_bluetooth_pairing" / "assets" / "phone.usda",
    "h06_button_pair": TASKS_DIR / "h06_bluetooth_pairing" / "assets" / "button_pair.usda",
    "h06_button_play": TASKS_DIR / "h06_bluetooth_pairing" / "assets" / "button_play.usda",
    "h06_button_power": TASKS_DIR / "h06_bluetooth_pairing" / "assets" / "button_power.usda",
    # built by tasks/l02_dj_mix/assets/build.py (original geometry and artwork)
    "l02_controller": TASKS_DIR / "l02_dj_mix" / "assets" / "controller.usda",
    "l02_play_button": TASKS_DIR / "l02_dj_mix" / "assets" / "play_button.usda",
}


def register() -> None:
    """Add every local model to Arena's asset registry (idempotent). Import after app start."""
    from isaaclab_arena.assets.object_library import LibraryObject
    from isaaclab_arena.assets.register import register_asset
    from isaaclab_arena.assets.registries import AssetRegistry

    for name, path in LOCAL.items():
        if AssetRegistry().is_registered(name, ensure_loaded=False):
            continue
        register_asset(type(f"Local_{name}", (LibraryObject,), {"name": name, "tags": ["object"], "usd_path": str(path)}))
