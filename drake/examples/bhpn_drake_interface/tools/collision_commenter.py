def comment_collisions(path):
    description = open(path, 'r')
    text = description.read()
    description.close()
    text = text.replace('<collision', '<!--collision')
    text = text.replace('</collision>', '</collision-->')
    p, e = path.split('.')
    p += '_no_collisions'
    new_path = p + '.' + e
    new_description = open(new_path, 'w')
    new_description.write(text)
    new_description.close()
