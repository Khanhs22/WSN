------ Node WSN group 11 -------
* Chương trình nạp trên kit ESP32 Devkit V1.

* Hướng dẫn nạp chương trình:
- Cài đặt và mở phần mềm ESP-IDF.
- Truy cập vào đường dẫn chứa file của từng project.
- Để build code, nhập lệnh "idf.py build".
- Để nạp code, nhập lệnh "idf.py -p (COM) flash".
- Để nạp code và theo dõi chương trình, nhập lệnh "idf.py -p (COM) flash monitor".
- Để tắt thông tin chương trình qua terminal, ấn "Ctrl + ]".

* Cách kết nối và chức năng các node:
- Khởi động node và chờ kết nối tới gateway, khi đèn xanh trên kit sáng -> kết nối thành công. Nếu kết nối không được, ta cần reset lại gateway và chờ đợi kết nối.
- Khi khởi động, giá trị nhiệt độ sẽ được hiển thị trên màn hình LDC. 
- Chu kì đo và ngưỡng nhiệt độ cao, thấp có giá trị lần lượt là 10s, 30 độ và 20 độ. Các giá trị này có thể được thay đổi thông qua cloud server, sau khi cài đặt sẽ hiển thị lên màn hình LCD của node. Tất cả dữ liệu đều được lưu trong bộ nhớ flash nên sẽ không bị mất khi mất điện hoặc hết pin.
- Khi giá trị nhiệt độ đo được thấp hơn mức low temp, đèn vàng sẽ sáng. Khi cao hơn mức high tem, đèn đỏ sẽ sáng. Khi ở mức bình thường, đèn xanh sẽ sáng.
- Giá trị đọc được từ cảm biến sẽ được gửi về gateway theo chu kì.