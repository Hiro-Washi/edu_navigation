from edu_navigation import app
from flask import render_template

@app.route('/')
def index():
    button = { 'name' : 'Launcher',
               'desc' : '機能の起動',
               'data' : 'any data'
    }
    return render_template('vt_index.html',
                            info1=button)
