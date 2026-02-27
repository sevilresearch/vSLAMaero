"""
Jason Carnahan
5/14/2021

Handles the logical decisions of the UAV.
"""


class Management:

    def __init__(self, parent):
        self.parent = parent
        # DEBUG counters for behaviour statistics 
        self.count_too_close = 0      # times we used "too close, move away"
        self.count_entropy_lt = 0     # times we used "entropy < threshold" branch
        self.count_entropy_ge = 0     # times we used "entropy >= threshold" branch