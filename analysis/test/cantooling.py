import cantools

if __name__ == '__main__':
    db = cantools.database.load_file('../data/hyundai_can.dbc')
    msg = db.get_message_by_frame_id(1056)
    
