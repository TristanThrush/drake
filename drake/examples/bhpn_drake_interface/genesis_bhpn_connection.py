from collections import Counter

m = {'drake_table': 'table', 'drake_soda': 'soda'} # TODO: get rid of this.

def place_string_from_number(number): # TODO: figure out better way.
    if number == 1:
        return 'first'
    if number == 2:
        return 'second'
    if number == 3:
        return 'third'
    if number == 4:
        return 'fourth'
    if number == 5:
        return 'fifth'
    if number == 6:
        return 'sixth'
    if number == 7:
        return 'seventh'
    if number == 8:
        return 'eigth'
    if number == 9:
        return 'ninth'
    if number == 10:
        return 'tenth'
    if number == 11:
        return 'eleventh'
    if number == 12:
        return 'twelfth'
    if number == 13:
        return 'thirteenth'
    if number == 14:
        return 'fourteenth'
    if number == 15:
        return 'fifteenth'
    else:
        raise ValueError('I do not know numbers as high as ' + str(number))


def operation_string_from_operation(operation, form_option=0):
    if form_option != 1 and form_option != 0:
        raise ValueError(str(form_option) + ' is not a form_option.')

    if operation.name == 'PlaceRel' or operation.name == 'Place':
        if form_option == 0:
            return 'put down the ' + m[operation.args[0]] # + ' on ' + operation.args[2]
        if form_option == 1:
            return 'putting down the ' + m[operation.args[0]] # + ' on ' + operation.args[2]
    elif operation.name == 'PickRel' or operation.name == 'Pick':
        if form_option == 0:
            return 'pick up the ' + m[operation.args[0]] # + ' from ' + operation.args[2]
        if form_option == 1:
            return 'picking up the ' + m[operation.args[0]] # + ' from ' + operation.args[2]
    elif operation.name == 'LookAt':
        if form_option == 0:
            return 'observe the ' + m[operation.args[0]]
        if form_option == 1:
            return 'observing the ' + m[operation.args[0]]
    elif operation.name == 'Move' or operation.name == 'MoveNB':
        if form_option == 0:
            return 'move '
        if form_option == 1:
            return 'moving '        
    else:
        raise ValueError('The operation type ' + operation.name + ' is not supported to be converted to a genesis story.')
    

def primitive_operation_explanation_story(ps):

    ps_guts_no_top = ps.guts()[1:]
    len_ps_guts_no_top = len(ps_guts_no_top)
    story_stack = []
    story_stack.append('') # For readability.
    previous_plan_current_operation = None
    for plan_index in range(len_ps_guts_no_top):

        plan = ps_guts_no_top[plan_index]
        steps_no_top = plan.steps[1:]
        operations = [step[0] for step in steps_no_top]
        len_operations = len(operations)
        operation_words = [operation_string_from_operation(operation) for operation in operations]
        operation_words_alternate_form = [operation_string_from_operation(operation, 1) for operation in operations]
        plan_number_word = place_string_from_number(plan_index+1)
        operation_words_with_time = [(operation_words[index], place_string_from_number(Counter(operation_words[:index+1])[operation_words[index]])) for index in range(len(operation_words))]
        operation_words_with_time_alternate_form = [(operation_words_alternate_form[index], place_string_from_number(Counter(operation_words_alternate_form[:index+1])[operation_words_alternate_form[index]])) for index in range(len(operation_words_alternate_form))]
        current_operation_index = plan.lastStepExecuted - 1

        if previous_plan_current_operation is None:
            # This is the first plan in the stack.
            plan_goal = 'complete the final goal'
        else:
            plan_goal = previous_plan_current_operation
        story_stack.append('I plan to ' +  plan_goal + ' because I want to ' + plan_goal + '.')
        if plan_index != len_ps_guts_no_top - 1:
            # If this is not the last plan in the stack.
            story_stack.append('I make my ' + plan_number_word + ' plan because I plan to ' + plan_goal + '.')
            story_stack.append('I want to complete the goals in my ' + plan_number_word + ' plan because I want to ' + plan_goal + '.')
            
            for operation_index in range(len_operations):
                story_stack.append(operation_words_with_time_alternate_form[operation_index][0].capitalize() + ' for the ' + operation_words_with_time_alternate_form[operation_index][1] + ' time comes ' + place_string_from_number(operation_index + 1) + ' in my ' + plan_number_word + ' plan because I make my ' + plan_number_word + ' plan.')
            
            if current_operation_index != 0:
                # If we are not at the very first goal of the plan
                story_stack.append('I want to complete the ' + place_string_from_number(current_operation_index + 1) + ' goal in my ' + plan_number_word + ' plan because I am at the ' + place_string_from_number(current_operation_index) + ' goal in my ' + plan_number_word + ' plan.')
            story_stack.append('I want to complete the ' + place_string_from_number(current_operation_index + 1) + ' goal in my ' + plan_number_word + ' plan because I have not arrived at the ' + place_string_from_number(current_operation_index + 1) + ' goal in my ' + plan_number_word + ' plan.')
            story_stack.append('I want to complete the ' + place_string_from_number(current_operation_index + 1) + ' goal in my ' + plan_number_word + ' plan because I want to complete the goals in my ' + plan_number_word + ' plan.')

            story_stack.append('I want to ' + operation_words_with_time[current_operation_index][0] + ' for the ' + operation_words_with_time[current_operation_index][1] + ' time in my ' + plan_number_word + ' plan because I want to complete the ' + place_string_from_number(current_operation_index + 1) + ' goal in my ' + plan_number_word + ' plan.')
            story_stack.append('I want to ' + operation_words_with_time[current_operation_index][0] + ' for the ' + operation_words_with_time[current_operation_index][1] + ' time in my ' + plan_number_word + ' plan because ' + operation_words_with_time_alternate_form[current_operation_index][0] + ' for the ' + operation_words_with_time_alternate_form[current_operation_index][1] + ' time comes ' + place_string_from_number(current_operation_index + 1) + ' in my ' + plan_number_word + ' plan.')
            previous_plan_current_operation = operation_words_with_time[current_operation_index][0] + ' for the ' + operation_words_with_time[current_operation_index][1] + ' time in my ' + plan_number_word + ' plan'
                        
        else:
            # If this is the last plan in the stack.
            story_stack.append('I cannot make a ' + plan_number_word + ' plan because I plan to ' + plan_goal + '.')
            story_stack.append('I cannot make a ' + plan_number_word + ' plan because I am confronted with a primitive goal.')

            story_stack.append('I ' + plan_goal + ' because I plan to ' + plan_goal + '.')
            story_stack.append('I ' + plan_goal + ' because I want to ' + plan_goal + '.')
            story_stack.append('I ' + plan_goal + ' because I cannot make a ' + plan_number_word + ' plan.')
        story_stack.append('') # Make a newline between each plan explanation for readability.    
    return story_stack

class StreamOfConsciousnessWriter:

    def __init__(self, output_directory):
        self.output_directory = output_directory
        self.stream_index_number = 0

    def write(self, story_stack):
        title = 'Stream of Index ' + str(self.stream_index_number)
        file_name = 'stream_of_index_' + str(self.stream_index_number) + '.txt'
        story_lines = ['Start story titled "' + title + '".'] + story_stack + ['The end.']
        f = open(self.output_directory + file_name, 'w')
        f.write('\n'.join(story_lines))
        f.close()
        self.stream_index_number += 1
        
        

   
