# پاسخ‌های فنی برای COMPAS Dev Exchange

## چرا به Forge نیاز داریم وقتی FAB برخورد دارد؟
FAB مدل، backend و برنامه‌ریزی دارد. Forge روی هندسه و مسیر پیشنهادی لایهٔ
preflight اضافه می‌کند: quality diagnostics، فاصله در طول حرکت، کران خطای
تقریب مفصلی، استفادهٔ مجدد از هندسه و خروجی صریح unknown. آن را می‌توان بعد
از تولید مسیر توسط backend موجود فراخواند.

## آیا COMPAS باواسطه است و Forge مستقیم؟
COMPAS هم در CPython راینو/گرس‌هاپر مستقیم اجرا می‌شود و PyBullet in-process
است. Forge برای محاسبات خودش سرویس خارجی لازم ندارد؛ Python/PyO3 و packing
داده همچنان وجود دارند. مزیت نصب native wheel در host را آزمایش کرده‌ایم.

## چرا Rust؟
برای snapshot مالکیت‌دار با طول عمر مشخص، queryهای هم‌زمانِ read-only، batch
و آزادشدن GIL در kernelهای سنگین. Rust به‌تنهایی دقت floating-point را بیشتر
نمی‌کند. الگوریتم، packing، FK و reuse هندسه هم روی زمان اثر دارند.

## آیا الگوریتم ریاضی جدید اختراع کرده‌اید؟
در وضعیت فعلی، ادعای نظریه یا الگوریتم بنیادی جدید نداریم. کار مهندسی ما
ترکیب قراردادها، adapter سازگار، bound و refinement و شواهد مستقل در این
گردش‌کار است. ادعای novelty پژوهشی به literature review و ارزیابی وسیع‌تر نیاز دارد.

## دقیقاً چه چیزی دو برابر سریع‌تر شده؟
فیلتر swept-AABB در یک مسیر UR5، روی همان build و با تکرارهای متناوب روشن/خاموش.
میانهٔ جدید ۱۲٫۰۵۴۱ در برابر ۲۴٫۲۴۴ ms، حدود ۲٫۰۱۱ برابر. این نتیجه برای همهٔ مش‌ها یا در برابر CGAL/
PyBullet نیست. preparation از این اندازه‌گیری کنار گذاشته شده و صریح گزارش شده.

## آیا ورودی واقعاً zero-copy است؟
در borrowed structural scan بافر آماده، کپی ورودی اضافی Rust نداریم. ساخت Mesh
از COMPAS packing دارد؛ retained collision mesh یک snapshot مالکیت‌دار است.
این مالکیت برای GIL-free/concurrent compute عمدی است.

## آیا clear تضمین ایمنی ربات است؟
clear یعنی در مدل، clearance و فرض‌های اعلام‌شده نقض پیدا/اثبات نشده و بازه‌ها
با کران مشروط جدا شده‌اند. کران عددی کل محاسبات، calibration، tracking، blending
و نیروهای واقعی به‌صورت اندازه‌گیری‌شده اثبات نشده‌اند. ماشین سیستم ایمنی مستقل دارد.

## چرا صرفاً تعداد نمونه‌ها را زیاد نمی‌کنید؟
نمونه‌های زیاد به‌تنهایی تضمین نمی‌کنند برخورد کوتاه بین آن‌ها دیده شود. کران
مشتق، خطای surrogate را محدود می‌کند؛ allowance مخصوص جفت و refinement محلی
تعداد محاسبات غیرضروری را کاهش می‌دهند. نزدیک مرز، unknown نتیجهٔ معتبر است.

## مرز خطای کران چیست؟
برای fixed-axis products و linear joint positions، استدلال حساب دقیق از B2 و
تغییر زاویه استفاده می‌کند. E/n² کران هندسی است. پیاده‌سازی f64 گواهی rounding
نیست؛ `numerical_error_bound_proven=False` همین مرز را اعلام می‌کند.

## آیا time_of_impact با شاهد clearance یکسان است؟
خیر. CCD می‌تواند TOI همراه وضعیت solver بدهد. شاهد clearance زمانی است که
فاصله از threshold کمتر شده؛ لزوماً اولین زمان عبور از threshold نیست. برای
حرکت articulated شاهد روی FK واقعی همان زمان دوباره بررسی می‌شود.

## اگر مش دیگری کاملاً داخل مش باشد چه می‌شود؟
surface distance ممکن است مثبت باشد. در solid mode، وضعیت آغازین محصورشدگی
بررسی می‌شود. آزمون CGAL نشان می‌دهد نداشتن تقاطع سطح با داشتن اشتراک جامد
سازگار است. بدون solid mode نتیجه فقط معنای سطحی دارد.

## چند پوسته و حفره را چطور معنی می‌کنید؟
single-shell پیش‌فرض جامد محدودتر است. opt-in even-odd پوسته‌های معتبر nested
را با parity تفسیر می‌کند. پوسته‌های intersecting یا union دلخواه، قرارداد دیگری دارند.

## mimic، انگشت ابزار و تغییر grasp پوشش دارند؟
mimic مستقیم مطابق precedence واقعی COMPAS، ابزار با per-link meshes و مسیر
هم‌زمان، و attachment phases با continuity checks پوشش دارند. TCP ثابت ToolModel
مطابق FAB است؛ به‌صورت خودکار به نوک انگشت دلخواه وصل نمی‌شود. grasp stability
یا باربری اتصال از continuity هندسی نتیجه نمی‌شود.

## trajectory velocities و accelerations چه نقشی دارند؟
ساختار و مقادیرشان اعتبارسنجی می‌شوند؛ بعضی حدود velocity هم کنترل می‌شوند.
مسیر هندسی فعلی خطی در joint positions است و Hermite/spline از derivatives
نمی‌سازد. کنترل average velocity جای dynamics یا jerk constraints را نمی‌گیرد.

## حذف جفت‌های مجاور و touch pairs چه خطری دارد؟
این‌ها policy ورودی و semantics هستند. clear برای مجموعهٔ جفت‌های فعال است؛
جفت مستثناشده بررسی نشده است. SRDF/touch exclusions باید با سلول واقعی بازبینی شوند.

## cached mesh بعد از تغییر هندسه چه می‌شود؟
snapshot تغییر خودکار نمی‌کند. باید instance جدید ساخته شود یا cache دوباره
ثبت شود. context manager و close، طول عمر منابع را مشخص می‌کنند.

## شواهد مستقل کدام‌اند؟
مرجع analytic، bounded least-squares و Bullet برای box-distance؛ CGAL برای
تقاطع سطح و جامد؛ Trimesh برای برخی predicates مش؛ property/adversarial tests.
دامنهٔ هر مرجع مشخص است. ۳۶ حالت CGAL یا ۱۲۰ box case، کل CCD صنعتی را اثبات نمی‌کند.

## همهٔ خانوادهٔ COMPAS سازگار است؟
APIهای اصلی با نسخه‌های تست‌شدهٔ COMPAS/FAB/Robots پوشش دارند. RRC 2 dependency
FAB کمتر از 2 دارد و در همین محیط سازگاری مستقیم اثبات نشده. IFC/BRep metadata،
controller و تمام افزونه‌ها adapter اختصاصی ندارند. Mesh مشترک می‌تواند نقطهٔ تبادل باشد.

## چه باگ upstream را حل کرده‌اید؟
برای هر ادعای bug باید reproducer و نسخهٔ دقیق داشته باشیم. فعلاً contribution
اصلی تکمیل گردش‌کار preflight است. در بازبینی خود Forge، اعتبارسنجی retained
joint values در partial trajectories اصلاح شد؛ آن را باگ COMPAS معرفی نمی‌کنیم.

## برای تبدیل‌شدن به contribution اصلی چه لازم است؟
بازبینی API توسط maintainer، fixtures واقعی، CI و host gates، benchmark هم‌معنا
و نگهداری نسخه‌ها. درخواست feedback روی همین قرارداد محدود و شواهد ثبت‌شده
مسیر همکاری روشنی ایجاد می‌کند.
