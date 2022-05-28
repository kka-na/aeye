# Boilerplate for DBC - https://cantools.readthedocs.io/en/latest/index.html
import cantools

if __name__ == '__main__':
    db = cantools.database.load_file('../data/hyundai_can.dbc')
    db.get_message_by_frame_id(id)
    db.get_message_by_name('SCC')

    db.decode_message()

