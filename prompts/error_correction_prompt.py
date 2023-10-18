# INPUT: [INSERT BLOCK NUMBER], [INSERT ERROR MESSAGE]
ERROR_CORRECTION_PROMPT = \
"""Running code block [INSERT BLOCK NUMBER] of your previous response resulted in the following error:
[INSERT ERROR MESSAGE]
Can you output a modified code block to resolve this error?
"""
