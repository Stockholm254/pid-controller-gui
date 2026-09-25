from main import run_app

if __name__ == '__main__':
    com_port = "COM50"  # Hardcoded, or load from config/env/argparse
    run_app(com_port)