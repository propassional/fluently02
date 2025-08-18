"""
    nlu
    ~~~

    The 'nlu' module provides the natural language understanding based on the 
    rasa framework.

    To test the node:
    ros2 run nlu nlu_node
    ros2 topic pub --once /nlu_input std_msgs/msg/String "{data: remove battery}
    "
"""
import time
import random
import os
import json
import yaml
import math
import warnings
import rclpy
import nltk
import asyncio
from spacy.lang.en import English
from nltk.tokenize import word_tokenize
from nltk import pos_tag
from quantulum3 import parser as units_parser
from rclpy.node import Node
from std_msgs.msg import String

nltk.download('punkt', quiet=True)
nltk.download('averaged_perceptron_tagger', quiet=True)
nltk.download('punkt_tab', quiet=True)
nltk.download('averaged_perceptron_tagger_eng', quiet=True)

os.environ["LOG_LEVEL"] = "INFO"  # rasa logging
os.environ["LOG_LEVEL_LIBRARIES"] = "INFO"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = '3'  # tensorflow logging
os.environ[
    "SQLALCHEMY_SILENCE_UBER_WARNING"] = '1'  # other logging environment variables

with warnings.catch_warnings():
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    from rasa.core.agent import Agent
    from rasa.utils.common import configure_logging_and_warnings
    from rasa.utils.log_utils import configure_structlog
    configure_logging_and_warnings()
    configure_structlog()

# path of the model
config_path = 'src/nlu/nlu/config.yaml'
with open(config_path, 'r') as yaml_file:
    yaml_content = yaml.safe_load(yaml_file)
rasa_model_path = yaml_content.get('model')

# divider
divider = "-" * 100 + "\n"

# intent_to_transition dictionary
# from src.nlu.nlu.sm_dictionary import intent_to_transition

class NluNode(Node):

    def __init__(self):

        # intent_to_transition dictionary
        intent_to_transition = {
            ("InstructionsStart", "Next"): "Next",
            ("InstructionsStart", "StateIsolated1"): "StateIsolated1",
            ("InstructionsStart", "AutopilotOn"): "AutopilotOn",
            ("InstructionsStart", "AutopilotOff"): "AutopilotOff",
            ("InstructionsStart", "MoveLeft"): "GoToTabLeft",
            ("InstructionsStart", "MoveRight"): "GoToTabRight",

            ("InstructionsReady", "Next"): "Next",
            ("InstructionsReady", "StateIsolated1"): "StateIsolated1",
            ("InstructionsReady", "AutopilotOn"): "AutopilotOn",
            ("InstructionsReady", "AutopilotOff"): "AutopilotOff",
            ("InstructionsReady", "MoveLeft"): "GoToTabLeft",
            ("InstructionsReady", "MoveRight"): "GoToTabRight",

            ("InstructionsRototiltInstalled", "Next"): "Next",
            ("InstructionsRototiltInstalled", "Last"): "Last",
            ("InstructionsRototiltInstalled", "StateIsolated1"): "StateIsolated1",
            ("InstructionsRototiltInstalled", "AutopilotOn"): "AutopilotOn",
            ("InstructionsRototiltInstalled", "AutopilotOff"): "AutopilotOff",
            ("InstructionsRototiltInstalled", "MoveLeft"): "GoToTabLeft",
            ("InstructionsRototiltInstalled", "MoveRight"): "GoToTabRight",

            ("InstructionsMarkersApplied", "Next"): "Next",
            ("InstructionsMarkersApplied", "Last"): "Last",
            ("InstructionsMarkersApplied", "StateIsolated1"): "StateIsolated1",
            ("InstructionsMarkersApplied", "AutopilotOn"): "AutopilotOn",
            ("InstructionsMarkersApplied", "AutopilotOff"): "AutopilotOff",
            ("InstructionsMarkersApplied", "MoveLeft"): "GoToTabLeft",
            ("InstructionsMarkersApplied", "MoveRight"): "GoToTabRight",

            ("InstructionsRingBelowScanned", "Next"): "Next",
            ("InstructionsRingBelowScanned", "Last"): "Last",
            ("InstructionsRingBelowScanned", "StateIsolated1"): "StateIsolated1",
            ("InstructionsRingBelowScanned", "AutopilotOn"): "AutopilotOn",
            ("InstructionsRingBelowScanned", "AutopilotOff"): "AutopilotOff",
            ("InstructionsRingBelowScanned", "MoveLeft"): "GoToTabLeft",
            ("InstructionsRingBelowScanned", "MoveRight"): "GoToTabRight",

            ("InstructionsRingIntermediateScanned", "Next"): "Next",
            ("InstructionsRingIntermediateScanned", "Last"): "Last",
            ("InstructionsRingIntermediateScanned", "StateIsolated1"): "StateIsolated1",
            ("InstructionsRingIntermediateScanned", "AutopilotOn"): "AutopilotOn",
            ("InstructionsRingIntermediateScanned", "AutopilotOff"): "AutopilotOff",
            ("InstructionsRingIntermediateScanned", "MoveLeft"): "GoToTabLeft",
            ("InstructionsRingIntermediateScanned", "MoveRight"): "GoToTabRight",

            ("InstructionsRingAboveScanned", "Next"): "Next",
            ("InstructionsRingAboveScanned", "Last"): "Last",
            ("InstructionsRingAboveScanned", "StateIsolated1"): "StateIsolated1",
            ("InstructionsRingAboveScanned", "AutopilotOn"): "AutopilotOn",
            ("InstructionsRingAboveScanned", "AutopilotOff"): "AutopilotOff",
            ("InstructionsRingAboveScanned", "MoveLeft"): "GoToTabLeft",
            ("InstructionsRingAboveScanned", "MoveRight"): "GoToTabRight",

            ("InstructionsEnd", "Next"): "Next",
            ("InstructionsEnd", "Last"): "Last",
            ("InstructionsEnd", "StateIsolated1"): "StateIsolated1",
            ("InstructionsEnd", "AutopilotOn"): "AutopilotOn",
            ("InstructionsEnd", "AutopilotOff"): "AutopilotOff",
            ("InstructionsEnd", "MoveLeft"): "GoToTabLeft",
            ("InstructionsEnd", "MoveRight"): "GoToTabRight",

            ("SetupStart", "Autopilot"): "Auto",
            ("SetupStart", "CameraSetupAdvanced"): "CameraSetupAdvanced",
            ("SetupStart", "CameraSetupBasic"): "CameraSetupBasic",
            ("SetupStart", "UserPreset"): "UserPreset",
            ("SetupStart", "AutopilotOn"): "AutopilotOn",
            ("SetupStart", "AutopilotOff"): "AutopilotOff",
            ("SetupStart", "MoveLeft"): "GoToTabLeft",
            ("SetupStart", "MoveRight"): "GoToTabRight",

            ("PreviewBasicStart", "Next"): "Next",
            ("PreviewBasicStart", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicStart", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicStart", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicStart", "MoveRight"): "GoToTabRight",

            ("PreviewBasicReady", "Check"): "CheckArtecStudioIsOff",
            ("PreviewBasicReady", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicReady", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicReady", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicReady", "MoveRight"): "GoToTabRight",

            ("PreviewBasicArtecStudioIsOff", "Start"): "StartArtecStudio",
            ("PreviewBasicArtecStudioIsOff", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicArtecStudioIsOff", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicArtecStudioIsOff", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicArtecStudioIsOff", "MoveRight"): "GoToTabRight",

            ("PreviewBasicArtecStudioIsOn", "Start"): "StartScanMenu",
            ("PreviewBasicArtecStudioIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicArtecStudioIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicArtecStudioIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicArtecStudioIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewBasicCameraSetupBasicVerified", "Next"): "Next",
            ("PreviewBasicCameraSetupBasicVerified", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicCameraSetupBasicVerified", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicCameraSetupBasicVerified", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicCameraSetupBasicVerified", "MoveRight"): "GoToTabRight",

            ("PreviewBasicCameraSetupAdvanced1Verified", "Next"): "Next",
            ("PreviewBasicCameraSetupAdvanced1Verified", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicCameraSetupAdvanced1Verified", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicCameraSetupAdvanced1Verified", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicCameraSetupAdvanced1Verified", "MoveRight"): "GoToTabRight",

            ("PreviewBasicCameraSetupAdvanced2Verified", "Start"): "StartPreview",
            ("PreviewBasicCameraSetupAdvanced2Verified", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicCameraSetupAdvanced2Verified", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicCameraSetupAdvanced2Verified", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicCameraSetupAdvanced2Verified", "MoveRight"): "GoToTabRight",

            ("PreviewBasicScanMenuIsOn", "Check"): "CheckScannerIsOn",
            ("PreviewBasicScanMenuIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicScanMenuIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicScanMenuIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicScanMenuIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewBasicScannerIsOn", "Next"): "Next",
            ("PreviewBasicScannerIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicScannerIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicScannerIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicScannerIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewBasicPreviewIsOn", "Autopilot"): "Auto",
            ("PreviewBasicPreviewIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicPreviewIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicPreviewIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicPreviewIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewBasicEnd", "Next"): "Next",
            ("PreviewBasicEnd", "AutopilotOn"): "AutopilotOn",
            ("PreviewBasicEnd", "AutopilotOff"): "AutopilotOff",
            ("PreviewBasicEnd", "MoveLeft"): "GoToTabLeft",
            ("PreviewBasicEnd", "MoveRight"): "GoToTabRight",

            ("PreviewStart", "Next"): "Next",
            ("PreviewStart", "AutopilotOn"): "AutopilotOn",
            ("PreviewStart", "AutopilotOff"): "AutopilotOff",
            ("PreviewStart", "MoveLeft"): "GoToTabLeft",
            ("PreviewStart", "MoveRight"): "GoToTabRight",

            ("PreviewReady", "Check"): "CheckArtecStudioIsOn",
            ("PreviewReady", "AutopilotOn"): "AutopilotOn",
            ("PreviewReady", "AutopilotOff"): "AutopilotOff",
            ("PreviewReady", "MoveLeft"): "GoToTabLeft",
            ("PreviewReady", "MoveRight"): "GoToTabRight",

            ("PreviewArtecStudioIsOn", "Start"): "StartScanMenu",
            ("PreviewArtecStudioIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewArtecStudioIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewArtecStudioIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewArtecStudioIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewScanMenuIsOn", "Check"): "CheckScannerIsOn",
            ("PreviewScanMenuIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewScanMenuIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewScanMenuIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewScanMenuIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewScannerIsOn", "Start"): "StartPreview",
            ("PreviewScannerIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewScannerIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewScannerIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewScannerIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewPreviewIsOn", "Autopilot"): "Auto",
            ("PreviewPreviewIsOn", "AutopilotOn"): "AutopilotOn",
            ("PreviewPreviewIsOn", "AutopilotOff"): "AutopilotOff",
            ("PreviewPreviewIsOn", "MoveLeft"): "GoToTabLeft",
            ("PreviewPreviewIsOn", "MoveRight"): "GoToTabRight",

            ("PreviewEnd", "Next"): "Next",
            ("PreviewEnd", "AutopilotOn"): "AutopilotOn",
            ("PreviewEnd", "AutopilotOff"): "AutopilotOff",
            ("PreviewEnd", "MoveLeft"): "GoToTabLeft",
            ("PreviewEnd", "MoveRight"): "GoToTabRight",

            ("FullScanStart", "Next"): "Next",
            ("FullScanStart", "SaveFile"): "SaveFile",
            ("FullScanStart", "AutopilotOn"): "AutopilotOn",
            ("FullScanStart", "AutopilotOff"): "AutopilotOff",
            ("FullScanStart", "MoveLeft"): "GoToTabLeft",
            ("FullScanStart", "MoveRight"): "GoToTabRight",

            ("FullScanReady", "Check"): "CheckPreview",
            ("FullScanReady", "SaveFile"): "SaveFile",
            ("FullScanReady", "AutopilotOn"): "AutopilotOn",
            ("FullScanReady", "AutopilotOff"): "AutopilotOff",
            ("FullScanReady", "MoveLeft"): "GoToTabLeft",
            ("FullScanReady", "MoveRight"): "GoToTabRight",

            ("FullScanPreviewIsOn", "Start"): "StartRecording",
            ("FullScanPreviewIsOn", "Stop"): "StopRecording",
            ("FullScanPreviewIsOn", "SaveFile"): "SaveFile",
            ("FullScanPreviewIsOn", "AutopilotOn"): "AutopilotOn",
            ("FullScanPreviewIsOn", "AutopilotOff"): "AutopilotOff",
            ("FullScanPreviewIsOn", "MoveLeft"): "GoToTabLeft",
            ("FullScanPreviewIsOn", "MoveRight"): "GoToTabRight",

            ("FullScanRecordingIsOn", "Start"): "StartPause",
            ("FullScanRecordingIsOn", "SaveFile"): "SaveFile",
            ("FullScanRecordingIsOn", "AutopilotOn"): "AutopilotOn",
            ("FullScanRecordingIsOn", "AutopilotOff"): "AutopilotOff",
            ("FullScanRecordingIsOn", "MoveLeft"): "GoToTabLeft",
            ("FullScanRecordingIsOn", "MoveRight"): "GoToTabRight",

            ("FullScanPauseIsOn", "Start"): "StartRecording",
            ("FullScanPauseIsOn", "Stop"): "StopRecording",
            ("FullScanPauseIsOn", "SaveFile"): "SaveFile",
            ("FullScanPauseIsOn", "AutopilotOn"): "AutopilotOn",
            ("FullScanPauseIsOn", "AutopilotOff"): "AutopilotOff",
            ("FullScanPauseIsOn", "MoveLeft"): "GoToTabLeft",
            ("FullScanPauseIsOn", "MoveRight"): "GoToTabRight",

            ("FullScanRecordingIsOff", "Start"): "StartPreview",
            ("FullScanRecordingIsOff", "SaveFile"): "SaveFile",
            ("FullScanRecordingIsOff", "AutopilotOn"): "AutopilotOn",
            ("FullScanRecordingIsOff", "AutopilotOff"): "AutopilotOff",
            ("FullScanRecordingIsOff", "MoveLeft"): "GoToTabLeft",
            ("FullScanRecordingIsOff", "MoveRight"): "GoToTabRight",

            ("SliceScanStart", "Next"): "Next",
            ("SliceScanStart", "SaveLowPosition"): "SaveLowPosition",
            ("SliceScanStart", "SaveHighPosition"): "SaveHighPosition",
            ("SliceScanStart", "Scan"): "DoSliceScan",
            ("SliceScanStart", "AutopilotOn"): "AutopilotOn",
            ("SliceScanStart", "AutopilotOff"): "AutopilotOff",
            ("SliceScanStart", "MoveLeft"): "GoToTabLeft",
            ("SliceScanStart", "MoveRight"): "GoToTabRight",

            ("SliceScanHome", "Next"): "Next",
            ("SliceScanHome", "MoveUp"): "MoveUp",
            ("SliceScanHome", "MoveDown"): "MoveDown",
            ("SliceScanHome", "SaveLowPosition"): "SaveLowPosition",
            ("SliceScanHome", "SaveHighPosition"): "SaveHighPosition",
            ("SliceScanHome", "Scan"): "DoSliceScan",
            ("SliceScanHome", "AutopilotOn"): "AutopilotOn",
            ("SliceScanHome", "AutopilotOff"): "AutopilotOff",
            ("SliceScanHome", "MoveLeft"): "GoToTabLeft",
            ("SliceScanHome", "MoveRight"): "GoToTabRight",

            ("SliceScanMovedUp", "MoveHome"): "MoveHome",
            ("SliceScanMovedUp", "MoveUp"): "MoveUp",
            ("SliceScanMovedUp", "MoveDown"): "MoveDown",
            ("SliceScanMovedUp", "SaveLowPosition"): "SaveLowPosition",
            ("SliceScanMovedUp", "SaveHighPosition"): "SaveHighPosition",
            ("SliceScanMovedUp", "Scan"): "DoSliceScan",
            ("SliceScanMovedUp", "AutopilotOn"): "AutopilotOn",
            ("SliceScanMovedUp", "AutopilotOff"): "AutopilotOff",
            ("SliceScanMovedUp", "MoveLeft"): "GoToTabLeft",
            ("SliceScanMovedUp", "MoveRight"): "GoToTabRight",

            ("SliceScanMovedDown", "MoveHome"): "MoveHome",
            ("SliceScanMovedDown", "MoveUp"): "MoveUp",
            ("SliceScanMovedDown", "MoveDown"): "MoveDown",
            ("SliceScanMovedDown", "SaveLowPosition"): "SaveLowPosition",
            ("SliceScanMovedDown", "SaveHighPosition"): "SaveHighPosition",
            ("SliceScanMovedDown", "Scan"): "DoSliceScan",
            ("SliceScanMovedDown", "AutopilotOn"): "AutopilotOn",
            ("SliceScanMovedDown", "AutopilotOff"): "AutopilotOff",
            ("SliceScanMovedDown", "MoveLeft"): "GoToTabLeft",
            ("SliceScanMovedDown", "MoveRight"): "GoToTabRight",

            ("SnakeScanStart", "Next"): "Next",
            ("SnakeScanStart", "SaveLowPosition"): "SaveLowPosition",
            ("SnakeScanStart", "SaveHighPosition"): "SaveHighPosition",
            ("SnakeScanStart", "Scan"): "DoPartialScan",
            ("SnakeScanStart", "EnlargeSnake"): "EnlargeSnake",
            ("SnakeScanStart", "ShrinkSnake"): "ShrinkSnake",
            ("SnakeScanStart", "AutopilotOn"): "AutopilotOn",
            ("SnakeScanStart", "AutopilotOff"): "AutopilotOff",
            ("SnakeScanStart", "MoveLeft"): "GoToTabLeft",
            ("SnakeScanStart", "MoveRight"): "GoToTabRight",

            ("SnakeScanReady", "Next"): "Next",
            ("SnakeScanReady", "SaveLowPosition"): "SaveLowPosition",
            ("SnakeScanReady", "SaveHighPosition"): "SaveHighPosition",
            ("SnakeScanReady", "Scan"): "DoPartialScan",
            ("SnakeScanReady", "EnlargeSnake"): "EnlargeSnake",
            ("SnakeScanReady", "ShrinkSnake"): "ShrinkSnake",
            ("SnakeScanReady", "AutopilotOn"): "AutopilotOn",
            ("SnakeScanReady", "AutopilotOff"): "AutopilotOff",
            ("SnakeScanReady", "MoveLeft"): "GoToTabLeft",
            ("SnakeScanReady", "MoveRight"): "GoToTabRight",

            ("SnakeScanHome", "MoveUp"): "MoveUp",
            ("SnakeScanHome", "MoveDown"): "MoveDown",
            ("SnakeScanHome", "SaveLowPosition"): "SaveLowPosition",
            ("SnakeScanHome", "SaveHighPosition"): "SaveHighPosition",
            ("SnakeScanHome", "Scan"): "DoPartialScan",
            ("SnakeScanHome", "EnlargeSnake"): "EnlargeSnake",
            ("SnakeScanHome", "ShrinkSnake"): "ShrinkSnake",
            ("SnakeScanHome", "AutopilotOn"): "AutopilotOn",
            ("SnakeScanHome", "AutopilotOff"): "AutopilotOff",
            ("SnakeScanHome", "MoveLeft"): "GoToTabLeft",
            ("SnakeScanHome", "MoveRight"): "GoToTabRight",

            ("SnakeScanMovedUp", "MoveUp"): "MoveUp",
            ("SnakeScanMovedUp", "MoveDown"): "MoveDown",
            ("SnakeScanMovedUp", "SaveLowPosition"): "SaveLowPosition",
            ("SnakeScanMovedUp", "SaveHighPosition"): "SaveHighPosition",
            ("SnakeScanMovedUp", "Scan"): "DoPartialScan",
            ("SnakeScanMovedUp", "EnlargeSnake"): "EnlargeSnake",
            ("SnakeScanMovedUp", "ShrinkSnake"): "ShrinkSnake",
            ("SnakeScanMovedUp", "AutopilotOn"): "AutopilotOn",
            ("SnakeScanMovedUp", "AutopilotOff"): "AutopilotOff",
            ("SnakeScanMovedUp", "MoveLeft"): "GoToTabLeft",
            ("SnakeScanMovedUp", "MoveRight"): "GoToTabRight",

            ("SnakeScanMovedDown", "MoveUp"): "MoveUp",
            ("SnakeScanMovedDown", "MoveDown"): "MoveDown",
            ("SnakeScanMovedDown", "SaveLowPosition"): "SaveLowPosition",
            ("SnakeScanMovedDown", "SaveHighPosition"): "SaveHighPosition",
            ("SnakeScanMovedDown", "Scan"): "DoPartialScan",
            ("SnakeScanMovedDown", "EnlargeSnake"): "EnlargeSnake",
            ("SnakeScanMovedDown", "ShrinkSnake"): "ShrinkSnake",
            ("SnakeScanMovedDown", "AutopilotOn"): "AutopilotOn",
            ("SnakeScanMovedDown", "AutopilotOff"): "AutopilotOff",
            ("SnakeScanMovedDown", "MoveLeft"): "GoToTabLeft",
            ("SnakeScanMovedDown", "MoveRight"): "GoToTabRight",

            ("RegistrationStart", "Next"): "Next",
            ("RegistrationStart", "AutopilotOn"): "AutopilotOn",
            ("RegistrationStart", "AutopilotOff"): "AutopilotOff",
            ("RegistrationStart", "MoveLeft"): "GoToTabLeft",
            ("RegistrationStart", "MoveRight"): "GoToTabRight",

            ("RegistrationReady", "Start"): "StartToolsMenu",
            ("RegistrationReady", "AutopilotOn"): "AutopilotOn",
            ("RegistrationReady", "AutopilotOff"): "AutopilotOff",
            ("RegistrationReady", "MoveLeft"): "GoToTabLeft",
            ("RegistrationReady", "MoveRight"): "GoToTabRight",

            ("RegistrationToolsMenuIsOn", "Start"): "StartToolsMenu",
            ("RegistrationToolsMenuIsOn", "AutopilotOn"): "AutopilotOn",
            ("RegistrationToolsMenuIsOn", "AutopilotOff"): "AutopilotOff",
            ("RegistrationToolsMenuIsOn", "MoveLeft"): "GoToTabLeft",
            ("RegistrationToolsMenuIsOn", "MoveRight"): "GoToTabRight",

            ("RegistrationAutoMenuIsOn", "Start"): "StartGlobalRegistration",
            ("RegistrationAutoMenuIsOn", "AutopilotOn"): "AutopilotOn",
            ("RegistrationAutoMenuIsOn", "AutopilotOff"): "AutopilotOff",
            ("RegistrationAutoMenuIsOn", "MoveLeft"): "GoToTabLeft",
            ("RegistrationAutoMenuIsOn", "MoveRight"): "GoToTabRight",

            ("RegistrationGlobalRegistrationEntered", "Autopilot"): "Auto",
            ("RegistrationGlobalRegistrationEntered", "AutopilotOn"): "AutopilotOn",
            ("RegistrationGlobalRegistrationEntered", "AutopilotOff"): "AutopilotOff",
            ("RegistrationGlobalRegistrationEntered", "MoveLeft"): "GoToTabLeft",
            ("RegistrationGlobalRegistrationEntered", "MoveRight"): "GoToTabRight",
            
            ("RegistrationCalculationIsDone", "Close"): "Close",
            ("RegistrationCalculationIsDone", "AutopilotOn"): "AutopilotOn",
            ("RegistrationCalculationIsDone", "AutopilotOff"): "AutopilotOff",
            ("RegistrationCalculationIsDone", "MoveLeft"): "GoToTabLeft",
            ("RegistrationCalculationIsDone", "MoveRight"): "GoToTabRight",

            ("RegistrationToolsMenuIsOff", "Next"): "Next",
            ("RegistrationToolsMenuIsOff", "AutopilotOn"): "AutopilotOn",
            ("RegistrationToolsMenuIsOff", "AutopilotOff"): "AutopilotOff",
            ("RegistrationToolsMenuIsOff", "MoveLeft"): "GoToTabLeft",
            ("RegistrationToolsMenuIsOff", "MoveRight"): "GoToTabRight",

            ("RegistrationEnd", "Stop"): "StopToolsMenu",
            ("RegistrationEnd", "AutopilotOn"): "AutopilotOn",
            ("RegistrationEnd", "AutopilotOff"): "AutopilotOff",
            ("RegistrationEnd", "MoveLeft"): "GoToTabLeft",
            ("RegistrationEnd", "MoveRight"): "GoToTabRight",

            ("CommandHubStart", "Autopilot"): "Auto",
            ("CommandHubStart", "Close"): "CloseAllArtecStudios",
            ("CommandHubStart", "Shutdown"): "Shutdown",
            ("CommandHubStart", "SaveFile"): "SaveFile",
            ("CommandHubStart", "AutopilotOn"): "AutopilotOn",
            ("CommandHubStart", "AutopilotOff"): "AutopilotOff",
            ("CommandHubStart", "MoveLeft"): "GoToTabLeft",
            ("CommandHubStart", "MoveRight"): "GoToTabRight"
        }

        self.intent_to_transition = intent_to_transition
        self.current_state = None # settiamo lo stato iniziale a None

        # answer structure
        self.feedback_questions = [
            "Did you ask me to {}?", "Do you want me to {}?", "Should I {}?",
            "Are you asking me to {}?", "{}, is this what you asked me to do?"
        ]
        self.intents_description = {
            'Check': ["check"],
            'Start': ["start"], 
            'Move': ["move"], 
            'Autopilot': ["autopilot"], 
            'Next': ["next"], 
            'Last': ["last"], 
            'StateIsolated1': ["state isolated"], 
            'CameraSetup': ["camera setup"], 
            'UserPreset': ["user preset"], 
            'Check': ["check"], 
            'Start': ["start"], 
            'Save': ["save"], 
            'Stop': ["stop"], 
            'Scan': ["scan"], 
            'EnlargeSnake': ["enlarge snake"], 
            'ShrinkSnake': ["shrink snake"], 
            'Close': ["close"], 
            'Shutdown': ["shutdown"],
            "Continue": ["continue", "proceed", "go on"],
            "RobotStop": ["firmly stop", "definitely stop", "halt"],
            "Pause": ["pause"],
            "GraspObject":
            ["pick up the item", "grasp the object", "get the object"],
            "PlaceObject": ["put down the item", "place the object apart"],
            "ReduceVelocity": ["slow down", "go slower", "reduce the speed"],
            "IncreaseVelocity":
            ["speed up", "go faster", "increase the velocity"],
            "ChangeVelocity": ["change the speed", "modify the velocity"],
            "MoveRobot": ["move"],
            "ChangeTool": ["get a new tool", "get another tool"],
            "ReturnHome": [
                "go to the default pose", "return to the home position",
                "go back to the starting pose"
            ],
            "SetHome":
            ["set a new home pose", "create a new default position"],
            "OpenGripper": ["open the gripper", "open the end effector"],
            "CloseGripper": ["close the gripper", "close the end effector"],
            "UnscrewN":
            ["remove the screws", "unscrew the bolts", "remove the bolts"],
            "ScanObject":
            ["learn a new battery pack", "scan a new batterz pack"],
            "SkillworxSlideChange": [
                "change the slide", "move to the next slide",
                "show the next step"
            ],
            "RemoveCells": [
                "remove the battery cells", "extract the cells",
                "take out the battery cells"
            ],
            "ActivateGripper":
            ["activate the gripper", "activate the tool", "setup the gripper"],
            "ActivateManualGuidance":
            ["activate the manual guidance", "go in hand guidance mode"],
            "ApproachObject": ["approach the object", "get closer to the object"],
            "TeachObject": ["teach the object", "learn the object"],
            "MoveJoint": ["move the joint", "change the joint position"],
        }

        # fallback management
        self.is_managing_fallback = False
        self.managing_output = None
        self.managing_iteration = 0

        # input node
        self.input_node = 'hfluently/PA_transcription'
        # self.input_node = 'nlu_input'

        self.data = ''
        self.negative_dictionary = ['', ' ']

        super().__init__('nlu_node')

        # publisher
        self.publisher_ = self.create_publisher(String, 'nlu_output', 10)

        # tts publisher
        self.tts_publisher = self.create_publisher(String, 'fluently/tts', 10)

        # simple intent publisher
        self.simple_intent_publisher = self.create_publisher(String, 'rfluently/simple_intent', 10)

        # gui publisher
        self.gui_publisher = self.create_publisher(String, 'gui_output', 10)

        # gui subscriber
        self.sm_state = 'sm_state'
        self.gui_subscription = self.create_subscription(String, self.sm_state,
                                                     self.gui_callback, 10)
        self.gui_subscription  

        # subscriber
        self.subscription = self.create_subscription(String, self.input_node,
                                                     self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # loading model
        self.policy = Agent.load(model_path=rasa_model_path)
        self.get_logger().info(f'Model loaded.\n\n{divider}')

    def CC_segmentation(self, sentence: str) -> list:
        """
        Perform segmentation based on conjuction in the sentence.

        :param sentence: sentence to segment
        :type l: str
        :returns: a list of single clauses in a sentence
        :rtype: list
        """
        words = word_tokenize(sentence)
        pos_tags = pos_tag(words)
        conjunctions = {'CC'}
        conjunction_positions = [
            i for i, tag in enumerate(pos_tags) if tag[1] in conjunctions
        ]

        if len(conjunction_positions) > 0:
            for pos in conjunction_positions:
                if pos > 0 and pos < len(words) - 1:
                    # POS tags: https://www.ling.upenn.edu/courses/Fall_2003/ling001/penn_treebank_pos.html
                    if pos_tags[pos + 1][1] in {
                            'PRP', 'VBZ', 'VB', 'VBP', 'VBD', 'VBG', 'VBN'
                    }:
                        part1 = ' '.join(words[:pos]) + ' '
                        part2 = ' '.join(words[pos + 1:])
                        return [str(part1), str(part2)]
                return [str(sentence)]
        else:
            return [str(sentence)]

    def segmentation(self, sentence: str) -> list:
        """
        :param sentence: sentence to segment
        :type l: str
        :returns: a list of single clauses in a sentence
        :rtype: list
        """
        nlp = English()
        config = {"punct_chars": []}
        # config=config is optional
        sentencizer = nlp.add_pipe("sentencizer", config=config)
        segmented_phrase = [str(i) for i in list(nlp(sentence).sents)]
        segmented_phrase_complete = []
        for segment in segmented_phrase:
            result = self.CC_segmentation(segment)
            segmented_phrase_complete += result

        # segmented_phrase is the result of the segmentation based on the spacy model
        # segmented_phrase_complete also includes the segmentation based on the conjunctions
        return segmented_phrase_complete

    def tidy_input(self, input):
        input = input.lower()
        input = input.strip(' ')
        if 'y' in input:
            return True
        else:
            return False

    def manage_fallback(self, input_text, data):
        """
        Evaluate the ranking if the intent has not been recognized. Ask the 
        operator whether the first valid option in the ranking is the one he was 
        looking for. Otherwise ask the which one was from the list and annotate 
        for data trainig.
        """
        if not self.is_managing_fallback:
            self.is_managing_fallback = True
            self.managing_output = data
            request_verbose = self.managing_output['intent_ranking'][
                self.managing_iteration]['name']
            message = random.choice(self.feedback_questions).format(
                random.choice(self.intents_description[request_verbose]))
            tts_output = {'message': message, 'timestamp': time.time()}

            # this will be displayed in skillworx
            self.get_logger().info('Message: "%s"\n' % message)
            self.managing_iteration += 1

            # publishing
            tts_msg = String()
            tts_msg.data = json.dumps(tts_output, indent=4)
            self.tts_publisher.publish(tts_msg)
            return None

        else:
            if data['intent'] == 'nlu_fallback' or data["entropy"] > 1:
                tts_output = {
                    'message': f"Could you repeat?",
                    'timestamp': time.time()
                }

                # this will be displayed in skillworx
                self.get_logger().info(f"Could you repeat?")

                # publishing
                tts_msg = String()
                tts_msg.data = json.dumps(tts_output, indent=4)
                self.tts_publisher.publish(tts_msg)
                return None

            elif data['intent'] == 'Confirmation':
                # originally there was no -1
                self.managing_output['intent'] = self.managing_output[
                    'intent_ranking'][self.managing_iteration - 1]['name']
                self.is_managing_fallback = False
                self.managing_iteration = 0
                return self.managing_output

            elif data['intent'] == 'Denial':
                if (self.managing_iteration > 3):
                    tts_output = {
                        'message': f"I am sorry, can you repeat the command?",
                        'timestamp': time.time()
                    }

                    # this will be displayed in skillworx
                    self.get_logger().info(
                        f"I am sorry, can you repeat the command?")

                    self.is_managing_fallback = False
                    self.managing_iteration = 0

                else:
                    request_verbose = self.managing_output['intent_ranking'][
                        self.managing_iteration]['name']
                    message = random.choice(self.feedback_questions).format(
                        random.choice(
                            self.intents_description[request_verbose]))
                    tts_output = {'message': message, 'timestamp': time.time()}

                    # this will be displayed in skillworx
                    self.get_logger().info('Message: \n"%s"\n' % message)
                    self.managing_iteration += 1

                tts_msg = String()
                tts_msg.data = json.dumps(tts_output, indent=4)
                self.tts_publisher.publish(tts_msg)
                return None
            else:
                self.is_managing_fallback = False
                self.managing_iteration = 0
                return data

        # # TODO: saving annotation to new training data
        # training_update_path = 'src/nlu/nlu/training_update.yaml'
        # with open(training_update_path, 'a') as file:
        #     yaml.dump({data['intent']: input_text}, file)

        return data

    def prepare_input(self, msg_data) -> str:
        '''
        Based on the name of the input topic, specific preprocessing are applied 
        to the input string.
        '''
        if self.input_node == '/whisper/text_output':
            msg_data = msg_data.strip('[whisper]:  ')
        elif self.input_node in '/hfluently/PA_transcription' or self.input_node in 'hfluently/PA_transcription':
            temp = json.loads(msg_data)
            msg_data = temp['transcription']

        return msg_data

    def valid_input(self, input_string):
        '''
        To prevent hallucinations to be assumed as valid input: check whether 
        input strings contain certain words, or if it is too long or short ...
        '''
        negative_dictionary = [
            'Bye. Bye.', 'Bye.', '', ' ', 'Bye-bye.', 'Thank you.'
        ]
        is_it_valid = (input_string not in negative_dictionary) and (
            len(input_string) < 75) and (len(input_string)
                                         > 1) and (not input_string.isspace())

        return is_it_valid

    def entropy(self, X) -> int:
        '''
        Given X, a list of probabilities, it compute entropy: 
        H(X) = -sum p(x)log(p(x)) 
        '''
        normX = [i / sum(X) for i in X]

        def h(x):
            return x * math.log(x, 2)

        hs = [h(i) for i in normX]
        H = -sum(hs)
        return H

    def digits_to_letters(self, data) -> str:
        '''
        It converts numbers written in digits to letters.
        '''
        # # TODO: this could be improved
        if '100' in data: # quantulum would parse it as one hundred
            data = data.replace('100', 'one-hundred')
        if 'to' in data: # quantulum parses ranges with average value e.g. 3 to 6 -> 4.5
            xs = [units_parser.inline_parse_and_replace(token) for token in data.split(' ')]
            data = ' '.join(xs)
        if any(char.isdigit() for char in data):
            data = units_parser.inline_parse_and_replace(data)
        return data
    
    def gui_callback(self, gui_msg):
        self.current_state = gui_msg.data
    
    def listener_callback(self, msg):

        # extract only relevant information from the input topic
        data = self.prepare_input(msg.data)

        # digits to letters conversion
        data = self.digits_to_letters(data)

        # TODO: 100 becomes one hundred. Make sure it is detected as one number
        self.get_logger().info('Input: "%s"\n' % data)

        # # multi intent recognition
        # multiple_intents = self.segmentation(data)
        # if len(multiple_intents) > 1:
        #     self.get_logger().info('Multiple intents: "%s"\n' %
        #                            str(multiple_intents))

        
        # for data in multiple_intents:

        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=UserWarning)

            # check for valid input string
            if self.valid_input(data):
                # output is a dictionary
                output = asyncio.run(self.policy.parse_message(data))
            else:
                output = {"intent": "not known"}

            # get results
            if output["intent"] != "not known":
                slim_output = {}
                slim_output['text'] = output['text']
                slim_output['intent'] = output['intent']['name']
                entities = []
                for entity in output['entities']:
                    slim_entity = {}
                    slim_entity['kind'] = entity['entity']
                    slim_entity['value'] = entity['value']
                    entities.append(slim_entity)
                slim_output['entities'] = entities
                slim_output['intent_ranking'] = output['intent_ranking']
                conf_list = []
                for intent in output['intent_ranking']:
                    if intent['name'] != 'nlu_fallback':
                        conf_list.append(intent['confidence'])
                slim_output['entropy'] = self.entropy(conf_list)

                # check for fallback
                if slim_output[
                        'entropy'] > 1 or self.is_managing_fallback:
                    slim_output['intent_ranking'] = [entry for entry in slim_output['intent_ranking'] if entry["name"] not in ["Confirmation", 
                    "Denial", "nlu_fallback"]]

                    self.get_logger().info(
                        'Uncertainty: \n"%s"\n' %
                        json.dumps(slim_output, indent=4))                        
                    slim_output = self.manage_fallback(
                        slim_output['text'], slim_output)

                if slim_output is not None:
                    if slim_output['intent'] == 'StartAudioRecording':
                        simple_intent_msg = {
                            'intent': f"StartAudioRecording",
                            'timestamp': time.time()
                        }
                        # publishing
                        simple_msg = String()
                        simple_msg.data = json.dumps(simple_intent_msg,
                                                        indent=4)
                        self.simple_intent_publisher.publish(simple_msg)
                    elif slim_output['intent'] == 'StopAudioRecording':
                        simple_intent_msg = {
                            'intent': f"StopAudioRecording",
                            'timestamp': time.time()
                        }
                        # publishing
                        simple_msg = String()
                        simple_msg.data = json.dumps(simple_intent_msg,
                                                        indent=4)
                        self.simple_intent_publisher.publish(simple_msg)
                    else:

                        # publishing
                        pub_msg = String()
                        pub_msg.data = json.dumps(slim_output, indent=4)
                        self.publisher_.publish(pub_msg)

                        print('\n')
                        self.get_logger().info('Output: \n"%s"\n' %
                                                pub_msg.data)
                        print(divider)



                        # GUI & state machine ------------------------------

                        # listen to state machine for current state -> we do it in gui_callback function
                        # these are strings e.g.  "InstructionsStart", "AutopilotOn"
                        current_state = self.current_state

                        # for testing use uncomment the following
                        # current_state = 'InstructionsStart'

                        intent_for_transition = slim_output['intent'] 
                        
                        # elaborate intent into transition
                        if len(slim_output['entities']) > 0:
                            entity_for_transition = slim_output['entities'][0]['value']
                        else:
                            entity_for_transition = ''
                        intent_plus_entity = intent_for_transition + entity_for_transition

                        if intent_plus_entity == 'Save':
                            intent_plus_entity = 'SaveFile'
                        elif intent_plus_entity == 'Move':
                            intent_plus_entity = 'MoveHome'                            

                        # TODO: check if the tuple is in the dictionary i.e. if the transition exists
                        key = (current_state, intent_plus_entity)
                        if key in self.intent_to_transition:
                            transition = self.intent_to_transition[key]
                            self.get_logger().info('Transition: \n"%s"\n' % transition)
                        
                            # publishing
                            msg_transition = String()
                            msg_transition.data = transition
                            self.gui_publisher.publish(pub_msg)
                        else:
                            self.get_logger().info('This transition is not possible: \n"%s"\n' % str(key))
                        # --------------------------------------------------

                            


def main(args=None):
    rclpy.init(args=args)
    nlu_node = NluNode()
    rclpy.spin(nlu_node)
    nlu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
