import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

class DiagnosticLogger:
    """
    DiagnosticLogger captures the lifecycle of items through the placement pipeline.
    It writes logs to individual files to be compatible with multiprocessing.
    """
    def __init__(self):
        self.log_dir = Path(os.environ.get('DIAGNOSTIC_LOG_DIR', 'results/diagnostic_logs'))
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def _log(self, item_id: Any, stage: str, data: Dict[str, Any]):
        # Filename format: item_{item_id}_{stage}.json
        # Use a string representation of item_id for the filename
        file_path = self.log_dir / f"item_{item_id}_{stage}.json"
        try:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"[DiagnosticLogger] Error writing log for item {item_id} at stage {stage}: {e}")

    def log_candidate_generation(self, item_id: Any, container_id: Any, candidates: List[Any]):
        """Observation Point 1: Candidate Generation"""
        # candidates is expected to be a list of (pos, ori) or similar
        self._log(item_id, "1_generation", {
            "container_id": container_id,
            "candidate_count": len(candidates),
            "candidates": candidates
        })

    def log_selected_candidate(self, item_id: Any, candidate_id: int, pos: Tuple[float, float, float], ori: int, score: float):
        """Observation Point 2: Candidate Selection"""
        self._log(item_id, "2_selection", {
            "candidate_id": candidate_id,
            "pos": pos,
            "ori": ori,
            "score": score
        })

    def log_cpp_check(self, item_id: Any, candidate_id: int, path_clear: bool, final_clear: bool):
        """Observation Point 3: C++ Checker"""
        self._log(item_id, "3_cpp", {
            "candidate_id": candidate_id,
            "path_clear": path_clear,
            "final_clear": final_clear
        })

    def log_physics(self, item_id: Any, planned_pos: Tuple[float, float, float], actual_pos: Tuple[float, float, float], delta_pos: float, delta_ori: float, settled: bool):
        """Observation Point 4: Physics/Placement"""
        self._log(item_id, "4_physics", {
            "planned_pos": planned_pos,
            "actual_pos": actual_pos,
            "delta_pos": delta_pos,
            "delta_ori": delta_ori,
            "settled": settled
        })

    def log_evaluation(self, item_id: Any, is_included: bool, is_inside: bool, hit_boundary: bool):
        """Observation Point 5: Evaluation"""
        self._log(item_id, "5_evaluation", {
            "is_included": is_included,
            "is_inside": is_inside,
            "hit_boundary": hit_boundary
        })

# Singleton instance
_logger = None

def get_logger() -> DiagnosticLogger:
    global _logger
    if _logger is None:
        _logger = DiagnosticLogger()
    return _logger
