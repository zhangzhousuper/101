1.提交格式正确
2.参数插值 /images/output1.png
  in rasterizer.cpp：修改函数 rasterize_triangle(const Triangle& t) 实现了法向量、颜色、纹理颜色的插值
  并将它们传递给 fragment_shader_payload
3. Blinn-phong 反射模型 /images/output2.png
  in main.cpp: 修改函数 phong_fragment_shader(): 实现 Blinn-Phong 模型计
  算 Fragment Color.
4. Texture mapping:  /images/output3.png
  in main.cpp: 修改函数 texture_fragment_shader()  在实现 Blinn-Phong
  的基础上，将纹理颜色视为公式中的 kd，实现 Texture Shading Fragment
  Shader
5.Bump mapping 与 Displacement mapping: /images/output4.png & output5.png
  使用TBN矩阵来处理表面高度
Bonus2  双线性纹理插值 /images/output-bil.png
