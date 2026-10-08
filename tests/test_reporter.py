"""HTML must render input strings as data, never executable markup."""
import pytest
from compas_forge.reporter import generate_html_report, _script_json


def test_html_report_escapes_text_and_script_data(tmp_path):
    payload = '</script><script>alert(1)</script>'
    output = tmp_path/'report.html'
    generate_html_report({'vertex_count':payload,'face_count':payload},
        {'profile_name':payload,'vertices':[[payload]],'genus':payload,'euler_characteristic':payload},
        {'welded_count':payload,'flipped_count':payload}, {payload:1.}, str(output))
    html = output.read_text(encoding='utf-8')
    assert payload not in html
    assert '&lt;/script&gt;' in html
    assert '\\u003c/script\\u003e' in html


@pytest.mark.parametrize('value',[float('nan'),float('inf')])
def test_script_json_rejects_nonfinite_numbers(value):
    with pytest.raises(ValueError):
        _script_json([value])
