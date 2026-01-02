# communication/logging_config.py
import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path

def setup_logging(log_file: str = "logs/communication.log", level=logging.INFO):
    Path(log_file).parent.mkdir(parents=True, exist_ok=True)

    logger = logging.getLogger()
    logger.setLevel(level)

    if logger.handlers:
        logger.handlers.clear()

    fh = RotatingFileHandler(log_file, maxBytes=5_000_000, backupCount=3, encoding="utf-8")
    fh.setLevel(level)
    fh_formatter = logging.Formatter("%(asctime)s | %(levelname)5s | %(name)s | %(message)s")
    fh.setFormatter(fh_formatter)
    logger.addHandler(fh)

    sh = logging.StreamHandler(stream=sys.stdout)
    sh.setLevel(level)
    sh_formatter = logging.Formatter("%(asctime)s | %(levelname)5s | %(name)s | %(message)s")
    sh.setFormatter(sh_formatter)
    logger.addHandler(sh)

    comm_logger = logging.getLogger("communication")
    comm_logger.setLevel(level)
    return logger
