
    def Copy_Paste(self, img, mask, img_name):
        img = img.convert("L")
        img_w, img_h = img.size
        # 获取所有含目标的源图像
        target_img_floder = "Dataset-mask/trainval/images"
        target_mask_floder = "Dataset-mask/trainval/masks"
        target_img_paths = sorted(
            [
                os.path.join(target_img_floder, img_name)
                for img_name in os.listdir(target_img_floder)
                if img_name.endswith(".png")
            ]
        )
        cp_num = 5
        result_mask = np.array(mask.copy())
        result_img = np.array(img.copy())
        for _ in range(cp_num):
            random_img_path = random.choice(target_img_paths)
            random_mask_path = os.path.join(
                target_mask_floder, os.path.basename(random_img_path)
            )
            target_img = cv2.imread(random_img_path, cv2.IMREAD_GRAYSCALE)
            target_mask = cv2.imread(random_mask_path, cv2.IMREAD_GRAYSCALE)

            # 查找轮廓
            contours, _ = cv2.findContours(
                target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            # 提取目标
            flag = 0
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                target = target_img[y : y + h, x : x + w]
                while True:
                    random_pos_y = random.randint(1, img_w - w - 1)
                    random_pos_x = random.randint(1, img_h - h - 1)
                    target_overlap = result_mask[
                        random_pos_x : random_pos_x + h, random_pos_y : random_pos_y + w
                    ].any()

                    if not target_overlap:
                        break
                    else:
                        flag += 1
                        if flag > 50:
                            break
                        else:
                            continue
                if flag > 50:
                    continue

                for i in range(h):
                    for j in range(w):
                        pixel_value = target[i, j]
                        result_img[random_pos_x + i, random_pos_y + j] = min(
                            255, pixel_value
                        )
                result_mask[
                    random_pos_x : random_pos_x + h, random_pos_y : random_pos_y + w
                ] = 255.0

        result_img = Image.fromarray(cv2.cvtColor(result_img, cv2.COLOR_GRAY2RGB))
        result_mask = Image.fromarray(result_mask)
        return result_img, result_mask
