from autodistill_grounding_dino import GroundingDINO
from autodistill.detection import CaptionOntology
from autodistill_yolov8 import YOLOv8
from autodistill.utils import plot
import os
from pathlib import Path
import shutil
import constants as constants

def main():
    if os.path.exists(constants.OUTPUT_FOLDER):
        response = input(f"Output folder {constants.OUTPUT_FOLDER} already exists. Delete and continue?").lower()
        if response == "y" or response == "yes":
            shutil.rmtree(constants.OUTPUT_FOLDER)
            print("removed folder")
        else:
            return
    base_model = GroundingDINO(ontology=CaptionOntology({"striped foam dodgeball": "ball"}), box_threshold=0.3, text_threshold=0.6)
    base_model.label(
        input_folder=constants.INPUT_FOLDER,
        extension=".jpg",
        output_folder=constants.OUTPUT_FOLDER
    )

if __name__ == "__main__":
    main()
