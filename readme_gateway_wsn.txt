------ Gateway WSN group 11 -------
* Chương trình nạp trên kit ESP32 Devkit V1.

* Hướng dẫn nạp chương trình:
- Cài đặt và mở phần mềm ESP-IDF.
- Truy cập vào đường dẫn chứa file của từng project.
- Để build code, nhập lệnh "idf.py build".
- Để nạp code, nhập lệnh "idf.py -p (COM) flash".
- Để nạp code và theo dõi chương trình, nhập lệnh "idf.py -p (COM) flash monitor".
- Để tắt thông tin chương trình qua terminal, ấn "Ctrl + ]".

* Hướng dẫn kết nối wifi của gateway (smart config):
- Tải phần mềm Esptouch trên điện thoại.
- Kết nối điện thoại với wifi đích cần kết nối cho esp32 (2.4Ghz).
- Mở ứng dụng Esptouch và nhập mật khẩu wifi.
- Đợi đến khi đèn xanh trên kit sáng => wifi đã được kết nối.
- Tên và mật khẩu wifi đã được lưu trên flash nên khi mất điện hoặc reset sẽ tự động kết nối lại và không bị mất dữ liệu.
- Để thay đổi wifi, ta ấn nút boot (GPIO0) trên kit, đèn xanh sẽ bị mất đi và kết nối lại wifi theo cách trên.

* Chức năng của gateway:
- Sau khi khởi động, sẽ khởi tạo BLE, wifi, kết nối với MQTT.
- Gateway được tích hợp chế độ smart-config, nên khi cấu hình wifi cho thiết bị ta không cần nhập wifi trong code, mà chỉ cần sử dụng phần mềm ESP Touch để cập nhật wifi. Không những vậy, trên gateway còn có 1 nút nhấn để xóa wifi cũ và tiến hành thay đổi wifi mới khi cần thiết, ta nhấn nút và sử dụng chế độ smart config như ban đầu. **
- Gateway sẽ quét các thiết bị trong vòng 30 giây, nếu quét được thiết bị đã có thông tin sẽ gửi yêu cầu kết nối và nhận bản tin quảng bá từ thiết bị đó (thời gian quét có thể thay đổi trong code).
- Các thiết bị đã có thông tin muốn kết nối bất kì lúc nào chỉ cần nhấn reset gateway, gateway sẽ quét thông tin và kết nối đến tất cả các thiết bị xung quanh đó (các thiết bị đã có thông tin trong gateway). **
- Khi nhận được thông tin từ các node, dữ liệu sẽ được gửi đến cloud (thingsboard.cloud) thông qua MQTT.
- Gateway được subcribe đến topic của cloud server, nên khi có cài đặt thay đổi chu kì đo và ngưỡng nhiệt độ, giá trị sẽ được gửi về gateway. Gateway nhận được dữ liệu sẽ xử lý chuỗi ký tự đó rồi gửi đến các node và tiến hành set up giá trị tại các node, lưu giá trị vào flash của các node. **