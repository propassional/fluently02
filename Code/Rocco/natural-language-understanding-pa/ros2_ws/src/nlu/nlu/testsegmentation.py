# import time
# import random
# import os
# import json
# import yaml
# import math
# import warnings
# import rclpy
import nltk
import spacy
from spacy.lang.en import English
from nltk.tokenize import word_tokenize
from nltk import pos_tag
# from quantulum3 import parser as units_parser
# from rclpy.node import Node
# from std_msgs.msg import String

nltk.download('punkt', quiet=True)
nltk.download('averaged_perceptron_tagger', quiet=True)
nltk.download('punkt_tab', quiet=True)
nltk.download('averaged_perceptron_tagger_eng', quiet=True)

def CC_segmentation(sentence: str) -> list:
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

def segmentation(sentence: str) -> list:
    """
    :param sentence: sentence to segment
    :type l: str
    :returns: a list of single clauses in a sentence
    :rtype: list
    """
    nlp = English()
    # config = {"punct_chars": [".", "?", "!", "。", ";", ","]}
    config = {"punct_chars": []}
    # config=config is optional
    sentencizer = nlp.add_pipe("sentencizer", config=config)
    segmented_phrase = [str(i) for i in list(nlp(sentence).sents)]
    segmented_phrase_complete = []
    for segment in segmented_phrase:
        result = CC_segmentation(segment)
        segmented_phrase_complete += result

    # segmented_phrase is the result of the segmentation based on the spacy model
    # segmented_phrase_complete also includes the segmentation based on the conjunctions
    return segmented_phrase_complete

print(segmentation("Good, thanks"))